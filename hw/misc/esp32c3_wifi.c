/*
 * ESP32-C3 WiFi MAC register block emulation
 *
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

/*
 * The C3 TRM documents no WiFi MAC at all, so as with the analog bus and the
 * front end there is nothing to model register by register.  What the block
 * needs is storage, so that a read returns the last write instead of the zero
 * the machine's catch-all I/O region returns, plus the handful of bits that
 * are status driven by the MAC rather than storage held for the driver.  A
 * store that only plays writes back reads those as zero forever and whatever
 * polls them never leaves the loop.
 *
 * The receive path is the exception: those registers are not guesses.  Each
 * one is the operand of a named accessor in the image's .wifi_iram section --
 * hal_mac_rx_set_base, hal_mac_interrupt_get_event and their neighbours are
 * two instructions each -- and the descriptor walk is spelled out in
 * wdevProcessRxSucDataAll in ROM.  esp32c3_wifi.h lists which name proves
 * which offset.
 *
 * esp32-open-mac's fork does the same for the ESP32 in hw/misc/esp32_wifi.c.
 * The offsets there are ESP32 offsets and do not carry over; these were found
 * from this image.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "exec/address-spaces.h"
#include "hw/misc/esp32c3_wifi.h"

/* Set to 1 to trace every access.  A wrong answer here produces no fault and
 * no unimplemented-register warning, so this is the only way to see what the
 * MAC was asked for; see the commit that added esp32c3_ana.c. */
#define ESP32C3_WIFI_DEBUG      0

/* Set to 1 to trace frame delivery: the descriptor taken, its buffer, and the
 * interrupt raised.  Separate from the above because the register trace is
 * hundreds of lines and this is three.  Off by default now that the access
 * point beacons: it is a line every hundred milliseconds, not three a run. */
#define ESP32C3_WIFI_RX_DEBUG   0

/* Set to 1 to trace the access point: what the station transmitted and what
 * was answered.  Left on -- it is one line per management frame and there are
 * four of them in an association, and it is what tells "the station never
 * asked" apart from "it asked and was not answered". */
#define ESP32C3_WIFI_AP_DEBUG   1

static void esp32c3_wifi_irq_update(ESP32C3WifiState *s)
{
    qemu_set_irq(s->irq, s->int_status != 0);
}

/*
 * The station's own address, as the guest programmed it.  Held in the register
 * file rather than in a field of its own so that it stays whatever the guest
 * last wrote: it is written once during bring-up and read on every frame.
 */
static const uint8_t *esp32c3_wifi_sta(ESP32C3WifiState *s)
{
    static uint8_t sta[6];

    stl_le_p(sta, s->mem[A_WIFI_STA_ADDR_LOW / sizeof(uint32_t)]);
    stw_le_p(sta + 4,
             (uint16_t) s->mem[A_WIFI_STA_ADDR_HIGH / sizeof(uint32_t)]);
    return sta;
}

static bool esp32c3_wifi_addr_is(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6) == 0;
}

/*
 * Write one frame into the descriptor the MAC would have used next and raise
 * the receive interrupt.  wdevProcessRxSucDataAll walks the list from the
 * descriptor the libraries remember, stopping at whichever one carries EOF or
 * at the address in A_WIFI_RX_DSCR_LAST, so both have to be right: EOF alone
 * with a stale "last" makes the walk run off the end of the ring, and a
 * "last" with no EOF anywhere makes it return having indicated nothing.
 */
static bool esp32c3_wifi_rx_deliver(ESP32C3WifiState *s, const uint8_t *frame,
                                    size_t len)
{
    static const uint8_t fcs[4] = { 0 };
    uint32_t dscr_addr = s->rx_dscr_next;
    uint32_t dscr[3];
    uint32_t flags, size, total, word0;
    uint8_t  hdr[WIFI_RX_CTRL_LEN];

    /* Before hal_mac_rx_set_base() runs there is no list to deliver into. */
    if (dscr_addr == 0) {
        return false;
    }

    address_space_read(&address_space_memory, dscr_addr, MEMTXATTRS_UNSPECIFIED,
                       dscr, sizeof(dscr));
    flags = le32_to_cpu(dscr[0]);
    size  = flags & WIFI_DSCR_SIZE_MASK;
    total = WIFI_RX_CTRL_LEN + len + sizeof(fcs);

    if (le32_to_cpu(dscr[1]) == 0 || total > size) {
        return false;
    }

    /*
     * wifi_pkt_rx_ctrl_t, built by hand because only six of its fields are
     * read on this path and the rest are reserved even in the public header.
     * The two match bits say which interface address the frame was addressed
     * to; with neither set wDev_ProcessRxSucData reaches wDev_DiscardFrame
     * before it reaches net80211, which looks exactly like a frame that never
     * arrived.  Addr1 decides it, so a beacon to the broadcast address has to
     * carry it as much as a data frame to the station does.
     */
    word0 = ((uint32_t) (uint8_t) -60)                         /* rssi, dBm */
            | (11u << 8)                                       /* rate      */
            | ((uint32_t) (len + sizeof(fcs)) << 16);          /* length    */
    if (len >= 10 && (esp32c3_wifi_addr_is(frame + 4, esp32c3_wifi_sta(s))
                      || (frame[4] & 1) != 0)) {
        word0 |= WIFI_RX_CTRL_STA_MATCH;
    }

    memset(hdr, 0, sizeof(hdr));
    stl_le_p(hdr + 0, word0);
    /*
     * The channel the frame was received on, twice.  Byte 10 is where
     * wDev_ProcessRxSucData stores it; byte 8 is where net80211 reads it, and
     * the two are not the same field.  scan_parse_beacon() compares the
     * channel in the DS parameter set against byte 8 and returns -1 when they
     * differ, so a beacon with byte 8 left at zero is parsed, found to
     * disagree with itself and dropped without a word -- the station scans,
     * finds nothing and disconnects, which is indistinguishable from an
     * access point that never answered.
     */
    hdr[8]  = (uint8_t) s->channel;
    hdr[10] = (uint8_t) s->channel;
    stl_le_p(hdr + 12, (uint32_t) (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000));
    hdr[20] = (uint8_t) -97;                                   /* noise     */
    stl_le_p(hdr + 44, (uint32_t) (len + sizeof(fcs))          /* sig_len   */
                       | ((uint32_t) (len + sizeof(fcs)) << 12)); /* copy,
                                                                  rx_state 0 */

    address_space_write(&address_space_memory, le32_to_cpu(dscr[1]),
                        MEMTXATTRS_UNSPECIFIED, hdr, sizeof(hdr));
    address_space_write(&address_space_memory,
                        le32_to_cpu(dscr[1]) + WIFI_RX_CTRL_LEN,
                        MEMTXATTRS_UNSPECIFIED, frame, len);
    /*
     * The four bytes of frame check sequence the radio would have received.
     * They are never right and never looked at -- rx_state above already says
     * the check passed -- but sig_len counts them, so net80211 takes the last
     * four bytes of whatever is here for the FCS and drops them.  Without
     * them it drops the last information element of every beacon instead.
     */
    address_space_write(&address_space_memory,
                        le32_to_cpu(dscr[1]) + WIFI_RX_CTRL_LEN + len,
                        MEMTXATTRS_UNSPECIFIED, fcs, sizeof(fcs));

    /*
     * The length the libraries believe is the descriptor's, not the header's:
     * wDev_ProcessRxSucData asserts on it before it does anything else.  The
     * owner bit goes back to software, as the DMA does when it retires a
     * descriptor.
     */
    flags &= ~(WIFI_DSCR_LENGTH_MASK | WIFI_DSCR_OWNER_HW);
    flags |= (total << WIFI_DSCR_LENGTH_SHIFT) & WIFI_DSCR_LENGTH_MASK;
    flags |= WIFI_DSCR_EOF;
    dscr[0] = cpu_to_le32(flags);
    address_space_write(&address_space_memory, dscr_addr, MEMTXATTRS_UNSPECIFIED,
                        dscr, sizeof(dscr[0]));

    s->mem[A_WIFI_RX_DSCR_LAST / sizeof(uint32_t)] = dscr_addr;
    s->mem[A_WIFI_RX_DSCR_NEXT / sizeof(uint32_t)] = le32_to_cpu(dscr[2]);
    s->rx_dscr_next = le32_to_cpu(dscr[2]);

#if ESP32C3_WIFI_RX_DEBUG
    info_report("[ESP32-C3][MAC] rx %zu bytes into descriptor 0x%08x "
                "buffer 0x%08x, next 0x%08x", len, dscr_addr,
                le32_to_cpu(dscr[1]), s->rx_dscr_next);
#endif

    s->int_status |= WIFI_INT_RX_SUC_DATA;
    esp32c3_wifi_irq_update(s);

    return true;
}

/*
 * The simulated access point.
 *
 * Everything below builds 802.11 frames by hand and hands them to the receive
 * path above, and reads the station's own frames out of the transmit queue.
 * The shape is esp32-open-mac's esp32_wifi_ap.c, which stock ESP-IDF
 * associates with; the frames transplant unchanged because they are 802.11 and
 * not Espressif, and only the registers around them had to be found again.
 *
 * The network says WPA2-PSK and means none of it.  Open was the intention and
 * would have been far less -- see esp32c3_wifi_ap_bss_body() for why this
 * image will not look at an open network -- but the privacy bit and an RSN
 * element are enough to get through the scan, and authentication and
 * association are both open-system either way.  What is genuinely missing is
 * the four-way handshake that follows, and esp32c3_wifi_ap_deauth() is where
 * that shows.
 */

#define IEEE80211_FC_MGMT_ASSOC_RESP    0x0010
#define IEEE80211_FC_MGMT_PROBE_RESP    0x0050
#define IEEE80211_FC_MGMT_BEACON        0x0080
#define IEEE80211_FC_MGMT_AUTH          0x00b0
#define IEEE80211_FC_MGMT_DEAUTH        0x00c0
#define IEEE80211_FC_DATA_FROM_DS       0x0208

#define IEEE80211_TYPE(fc)              (((fc) >> 2) & 0x3)
#define IEEE80211_SUBTYPE(fc)           (((fc) >> 4) & 0xf)
#define IEEE80211_TYPE_MGMT             0
#define IEEE80211_TYPE_DATA             2
#define IEEE80211_SUBTYPE_ASSOC_REQ     0
#define IEEE80211_SUBTYPE_REASSOC_REQ   2
#define IEEE80211_SUBTYPE_PROBE_REQ     4
#define IEEE80211_SUBTYPE_AUTH          11
#define IEEE80211_SUBTYPE_DEAUTH        12
#define IEEE80211_SUBTYPE_DISASSOC      10

#define IEEE80211_ELEM_SSID             0
#define IEEE80211_ELEM_RATES            1
#define IEEE80211_ELEM_DSPARMS          3
#define IEEE80211_ELEM_TIM              5
#define IEEE80211_ELEM_RSN              48
#define IEEE80211_ELEM_XRATES           50

static const uint8_t esp32c3_wifi_broadcast[6] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/* 1, 2, 5.5 and 11 Mbit/s basic, then the OFDM rates the station offered. */
static const uint8_t esp32c3_wifi_rates[]  = {
    0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24
};
static const uint8_t esp32c3_wifi_xrates[] = { 0x30, 0x48, 0x60, 0x6c };

/*
 * WPA2-PSK: CCMP for both the group and the pairwise cipher, PSK for the
 * authentication and key management.  The station will not look at an open
 * network -- see esp32c3_wifi_ap_bss_body() -- so this is not optional, and
 * what it buys is only the privacy bit and this element: nothing here derives
 * a key or encrypts anything, and the four-way handshake that follows
 * association is not answered.
 */
static const uint8_t esp32c3_wifi_rsn[] = {
    0x01, 0x00,                         /* version 1                        */
    0x00, 0x0f, 0xac, 0x04,             /* group cipher: CCMP               */
    0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, /* one pairwise cipher: CCMP        */
    0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, /* one AKM: PSK                     */
    0x00, 0x00                          /* RSN capabilities                 */
};

/*
 * Hand one built frame to the station, later.  Not now: rx_deliver() takes a
 * descriptor each time, and answering a probe request from inside the write
 * that transmitted it would put the reply in the ring before the station's own
 * transmit-complete handler had run.  One frame per timer tick also keeps the
 * ring from being lapped while the libraries are still walking it.
 */
static void esp32c3_wifi_ap_send(ESP32C3WifiState *s, const uint8_t *frame,
                                 size_t len)
{
    ESP32C3WifiFrame *f;

    if (len > sizeof(f->data)) {
        return;
    }

    f = g_new0(ESP32C3WifiFrame, 1);
    f->len = (uint16_t) len;
    memcpy(f->data, frame, len);

    if (s->rx_tail != NULL) {
        s->rx_tail->next = f;
    } else {
        s->rx_head = f;
    }
    s->rx_tail = f;

    timer_mod(s->rx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCALE_MS);
}

static void esp32c3_wifi_ap_pump(void *opaque)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(opaque);
    ESP32C3WifiFrame *f = s->rx_head;

    if (f == NULL) {
        return;
    }

    /*
     * A failure here is the ring being full, not the frame being bad, so the
     * frame stays at the head and is tried again rather than dropped: the
     * station is about to hand the buffers back through the reload handshake.
     */
    if (esp32c3_wifi_rx_deliver(s, f->data, f->len)) {
        s->rx_head = f->next;
        if (s->rx_head == NULL) {
            s->rx_tail = NULL;
        }
        g_free(f);
    }

    if (s->rx_head != NULL) {
        timer_mod(s->rx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCALE_MS);
    }
}

static uint8_t *esp32c3_wifi_ap_hdr(ESP32C3WifiState *s, uint8_t *p,
                                    uint16_t fc, const uint8_t *addr1)
{
    stw_le_p(p, fc);                p += 2;
    stw_le_p(p, 0);                 p += 2;   /* duration          */
    memcpy(p, addr1, 6);            p += 6;   /* receiver          */
    memcpy(p, s->bssid, 6);         p += 6;   /* transmitter       */
    memcpy(p, s->bssid, 6);         p += 6;   /* BSSID, or source  */
    stw_le_p(p, s->seq << 4);       p += 2;   /* sequence control  */
    s->seq = (s->seq + 1) & 0xfff;
    return p;
}

static uint8_t *esp32c3_wifi_ap_elem(uint8_t *p, uint8_t id, const void *value,
                                     size_t len)
{
    *p++ = id;
    *p++ = (uint8_t) len;
    memcpy(p, value, len);
    return p + len;
}

/*
 * The fixed fields and information elements a beacon and a probe response
 * share.
 *
 * The network has to say it is encrypted, which was not the plan.  An open
 * network is far less to model -- no key exchange at all -- and the received
 * wisdom is that a station configured with a password associates with one
 * anyway, because ESP-IDF's default threshold.authmode is OPEN.  This image
 * does not: scan_parse_beacon() tests the profile password against the
 * privacy bit of every candidate directly, before any threshold is consulted,
 *
 *   wifi_sta_get_prof_password()          -- non-empty, so
 *   lhu a5,6(s0) ; andi a5,a5,16          -- the privacy bit, and if clear
 *   wifi_log( "Open AP, but we want an encrypted AP, ignore" )
 *   g_authmode_incompatible = 1 ; return -1
 *
 * and the access point is dropped without appearing in the scan results.  So
 * the privacy bit is set and an RSN element says WPA2-PSK.  Nothing behind it
 * is real: see esp32c3_wifi_ap_deauth().
 */
static uint8_t *esp32c3_wifi_ap_bss_body(ESP32C3WifiState *s, uint8_t *p)
{
    uint8_t channel = (uint8_t) s->channel;

    stq_le_p(p, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000); p += 8;
    stw_le_p(p, 100);                                          p += 2;
    stw_le_p(p, s->privacy ? 0x0011 : 0x0001);                 p += 2;

    p = esp32c3_wifi_ap_elem(p, IEEE80211_ELEM_SSID, s->ssid, strlen(s->ssid));
    p = esp32c3_wifi_ap_elem(p, IEEE80211_ELEM_RATES, esp32c3_wifi_rates,
                             sizeof(esp32c3_wifi_rates));
    p = esp32c3_wifi_ap_elem(p, IEEE80211_ELEM_DSPARMS, &channel, 1);
    if (s->rsn) {
        p = esp32c3_wifi_ap_elem(p, IEEE80211_ELEM_RSN, esp32c3_wifi_rsn,
                                 sizeof(esp32c3_wifi_rsn));
    }
    p = esp32c3_wifi_ap_elem(p, IEEE80211_ELEM_XRATES, esp32c3_wifi_xrates,
                             sizeof(esp32c3_wifi_xrates));
    return p;
}

static void esp32c3_wifi_ap_beacon(void *opaque)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(opaque);
    uint8_t  frame[192];
    uint8_t *p = frame;

    /*
     * Only when the station is listening on this channel.  Nothing in the MAC
     * window says which channel that is -- the channel goes to the PHY, not
     * here -- but the station stamps it into the DS parameter set of every
     * probe request it sends, and once it has associated it stays on ours.
     * Beaconing regardless would put this network on every channel of the
     * scan, which is a worse lie than being quiet.
     */
    if (s->ap_state != ESP32C3_AP_ASSOCIATED && s->sta_channel != s->channel) {
        goto again;
    }

    p = esp32c3_wifi_ap_hdr(s, p, IEEE80211_FC_MGMT_BEACON,
                            esp32c3_wifi_broadcast);
    p = esp32c3_wifi_ap_bss_body(s, p);
    p = esp32c3_wifi_ap_elem(p, IEEE80211_ELEM_TIM, "\0\1\0\0", 4);
    esp32c3_wifi_ap_send(s, frame, p - frame);

again:
    timer_mod(s->beacon_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100 * SCALE_MS);
}

/* The information element with this identifier, or NULL. */
static const uint8_t *esp32c3_wifi_elem(const uint8_t *body, size_t len,
                                        uint8_t id)
{
    size_t i = 0;

    while (i + 2 <= len) {
        size_t elen = body[i + 1];

        if (i + 2 + elen > len) {
            break;
        }
        if (body[i] == id) {
            return body + i;
        }
        i += 2 + elen;
    }
    return NULL;
}

static void esp32c3_wifi_ap_probe_resp(ESP32C3WifiState *s, const uint8_t *sta)
{
    uint8_t  frame[160];
    uint8_t *p = frame;

    p = esp32c3_wifi_ap_hdr(s, p, IEEE80211_FC_MGMT_PROBE_RESP, sta);
    p = esp32c3_wifi_ap_bss_body(s, p);
    esp32c3_wifi_ap_send(s, frame, p - frame);
}

static void esp32c3_wifi_ap_auth_resp(ESP32C3WifiState *s, const uint8_t *sta)
{
    uint8_t  frame[64];
    uint8_t *p = frame;

    p = esp32c3_wifi_ap_hdr(s, p, IEEE80211_FC_MGMT_AUTH, sta);
    stw_le_p(p, 0);  p += 2;        /* open system, the only algorithm here */
    stw_le_p(p, 2);  p += 2;        /* transaction sequence                 */
    stw_le_p(p, 0);  p += 2;        /* status: successful                   */
    esp32c3_wifi_ap_send(s, frame, p - frame);
}

static void esp32c3_wifi_ap_assoc_resp(ESP32C3WifiState *s, const uint8_t *sta)
{
    uint8_t  frame[128];
    uint8_t *p = frame;

    p = esp32c3_wifi_ap_hdr(s, p, IEEE80211_FC_MGMT_ASSOC_RESP, sta);
    stw_le_p(p, s->privacy ? 0x0011 : 0x0001);
    p += 2;                         /* capability, as in the beacon         */
    stw_le_p(p, 0);       p += 2;   /* status: successful                   */
    stw_le_p(p, 0xc001);  p += 2;   /* association identifier 1             */
    p = esp32c3_wifi_ap_elem(p, IEEE80211_ELEM_RATES, esp32c3_wifi_rates,
                             sizeof(esp32c3_wifi_rates));
    p = esp32c3_wifi_ap_elem(p, IEEE80211_ELEM_XRATES, esp32c3_wifi_xrates,
                             sizeof(esp32c3_wifi_xrates));
    esp32c3_wifi_ap_send(s, frame, p - frame);
}

/*
 * The access point giving up on a station that never finished the key
 * exchange, with reason 15, four-way handshake timeout.
 *
 * This is not a corner case here, it is the normal end of every run: the
 * network advertises WPA2-PSK because the station will not look at an open
 * one, and nothing in this model derives a key, so the handshake that
 * association is supposed to be followed by never happens.  A real access
 * point deauthenticates on that timeout, and doing the same is what lets the
 * station reach a verdict instead of waiting out the example's five seconds
 * still trying -- "the connection attempt reached a verdict" is the check
 * that would otherwise fail, and it would fail for a reason that has nothing
 * to do with what it is testing.
 */
static void esp32c3_wifi_ap_deauth(void *opaque)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(opaque);
    uint8_t  frame[32];
    uint8_t *p = frame;

    if (s->ap_state != ESP32C3_AP_ASSOCIATED) {
        return;
    }

#if ESP32C3_WIFI_AP_DEBUG
    info_report("[ESP32-C3][AP] no four-way handshake, deauthenticating");
#endif

    p = esp32c3_wifi_ap_hdr(s, p, IEEE80211_FC_MGMT_DEAUTH,
                            esp32c3_wifi_sta(s));
    stw_le_p(p, 15);  p += 2;       /* four-way handshake timeout           */
    esp32c3_wifi_ap_send(s, frame, p - frame);
    s->ap_state = ESP32C3_AP_IDLE;
}

static uint16_t esp32c3_wifi_cksum(const uint8_t *p, size_t len)
{
    uint32_t sum = 0;
    size_t   i;

    for (i = 0; i + 1 < len; i += 2) {
        sum += ((uint32_t) p[i] << 8) | p[i + 1];
    }
    if (i < len) {
        sum += (uint32_t) p[i] << 8;
    }
    while ((sum >> 16) != 0) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return (uint16_t) ~sum;
}

/*
 * A frame for the station now that it has one to receive.
 *
 * The first is an ARP request for the address the example configures, and the
 * rest are ICMP echo requests from the same host.  Both are chosen so that a
 * working stack answers them: what the netif counters cannot show on their own
 * is whether a delivered frame was understood, and a reply coming back out
 * through the transmit queue says it was.
 */
static size_t esp32c3_wifi_make_frame(ESP32C3WifiState *s, uint8_t *buf,
                                      size_t buflen)
{
    static const uint8_t peer_ip[4] = { 10, 0, 2, 2 };
    static const uint8_t sta_ip[4]  = { 10, 0, 2, 15 };
    const uint8_t *sta = esp32c3_wifi_sta(s);
    uint8_t       *p = buf;
    uint8_t       *llc;

    assert(buflen >= 128);

    p = esp32c3_wifi_ap_hdr(s, p, IEEE80211_FC_DATA_FROM_DS, sta);

    llc = p;
    *p++ = 0xaa; *p++ = 0xaa; *p++ = 0x03;              /* LLC              */
    *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;              /* SNAP, RFC 1042   */

    if (s->injected == 0) {
        *p++ = 0x08; *p++ = 0x06;                       /* ARP              */
        *p++ = 0x00; *p++ = 0x01;                       /* Ethernet         */
        *p++ = 0x08; *p++ = 0x00;                       /* IPv4             */
        *p++ = 0x06; *p++ = 0x04;
        *p++ = 0x00; *p++ = 0x01;                       /* request          */
        memcpy(p, s->bssid, 6);   p += 6;               /* sender hardware  */
        memcpy(p, peer_ip, 4);    p += 4;
        memset(p, 0, 6);          p += 6;               /* target hardware  */
        memcpy(p, sta_ip, 4);     p += 4;
    } else {
        uint8_t *ip = p + 2;
        uint8_t *icmp;

        *p++ = 0x08; *p++ = 0x00;                       /* IPv4             */
        *p++ = 0x45; *p++ = 0x00;                       /* version, DSCP    */
        *p++ = 0x00; *p++ = 0x00;                       /* total length     */
        *p++ = 0x00; *p++ = 0x01;                       /* identification   */
        *p++ = 0x00; *p++ = 0x00;                       /* fragmentation    */
        *p++ = 0x40; *p++ = 0x01;                       /* TTL, ICMP        */
        *p++ = 0x00; *p++ = 0x00;                       /* header checksum  */
        memcpy(p, peer_ip, 4);    p += 4;
        memcpy(p, sta_ip, 4);     p += 4;

        icmp = p;
        *p++ = 0x08; *p++ = 0x00;                       /* echo request     */
        *p++ = 0x00; *p++ = 0x00;                       /* checksum         */
        stw_be_p(p, 0x1234);      p += 2;               /* identifier       */
        stw_be_p(p, s->injected); p += 2;               /* sequence         */
        memset(p, 'a', 16);       p += 16;

        stw_be_p(icmp + 2, esp32c3_wifi_cksum(icmp, p - icmp));
        stw_be_p(ip + 2, (uint16_t) (p - ip));
        stw_be_p(ip + 10, esp32c3_wifi_cksum(ip, 20));
    }

    /* An Ethernet frame is at least 60 bytes and lwIP is entitled to say so. */
    while (p - llc < 6 + 46) {
        *p++ = 0;
    }

    return p - buf;
}

/*
 * Deliver on a timer once the station has associated.  Before that the frames
 * would reach sta_input() and be dropped there, correctly, for want of a BSS
 * they could have come from -- which is where they stopped before this access
 * point existed.  Set inject-frames to 0 to turn it off.
 */
static void esp32c3_wifi_inject(void *opaque)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(opaque);
    uint8_t frame[256];
    size_t  len;

    if (s->ap_state != ESP32C3_AP_ASSOCIATED) {
        return;
    }

    len = esp32c3_wifi_make_frame(s, frame, sizeof(frame));
    esp32c3_wifi_ap_send(s, frame, len);
    s->injected++;

#if ESP32C3_WIFI_AP_DEBUG
    info_report("[ESP32-C3][AP] data frame %u of %u for the station",
                s->injected, s->inject_frames);
#endif

    if (s->injected < s->inject_frames) {
        timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                  + (int64_t) s->inject_period_ms * SCALE_MS);
    }
}

/*
 * One frame the station transmitted, with the frame check sequence already
 * removed.  Only the management subtypes that association is made of are
 * answered; a data frame is the station replying to what was injected above,
 * and is reported rather than routed because there is no netdev behind this
 * device for it to be routed to.
 */
static void esp32c3_wifi_ap_input(ESP32C3WifiState *s, const uint8_t *frame,
                                  size_t len)
{
    const uint8_t *sta;
    const uint8_t *body;
    const uint8_t *elem;
    size_t         body_len;
    uint16_t       fc;

    if (len < 24) {
        return;
    }

    fc   = lduw_le_p(frame);
    sta  = frame + 10;
    body = frame + 24;
    body_len = len - 24;

    if (IEEE80211_TYPE(fc) == IEEE80211_TYPE_DATA) {
#if ESP32C3_WIFI_AP_DEBUG
        info_report("[ESP32-C3][AP] the station transmitted %zu bytes of data",
                    len);
#endif
        return;
    }

    if (IEEE80211_TYPE(fc) != IEEE80211_TYPE_MGMT) {
        return;
    }

    switch (IEEE80211_SUBTYPE(fc)) {
    case IEEE80211_SUBTYPE_PROBE_REQ:
        /*
         * Which channel the station is on, which nothing else here can say.
         * Answering a probe request sent on another channel would be a frame
         * the radio could not have heard.
         */
        elem = esp32c3_wifi_elem(body, body_len, IEEE80211_ELEM_DSPARMS);
        if (elem != NULL && elem[1] == 1) {
            s->sta_channel = elem[2];
        }

        elem = esp32c3_wifi_elem(body, body_len, IEEE80211_ELEM_SSID);
        if (elem != NULL && elem[1] != 0
            && (elem[1] != strlen(s->ssid)
                || memcmp(elem + 2, s->ssid, elem[1]) != 0)) {
            return;
        }
        if (s->sta_channel != 0 && s->sta_channel != s->channel) {
            return;
        }
#if ESP32C3_WIFI_AP_DEBUG
        info_report("[ESP32-C3][AP] probe request on channel %u, answering",
                    s->sta_channel);
#endif
        esp32c3_wifi_ap_probe_resp(s, sta);
        break;

    case IEEE80211_SUBTYPE_AUTH:
        /* Open system, transaction one.  Anything else is not this network. */
        if (body_len < 6 || lduw_le_p(body) != 0 || lduw_le_p(body + 2) != 1) {
            return;
        }
#if ESP32C3_WIFI_AP_DEBUG
        info_report("[ESP32-C3][AP] authentication request, accepting");
#endif
        s->ap_state = ESP32C3_AP_AUTHENTICATED;
        esp32c3_wifi_ap_auth_resp(s, sta);
        break;

    case IEEE80211_SUBTYPE_ASSOC_REQ:
    case IEEE80211_SUBTYPE_REASSOC_REQ:
        if (s->ap_state == ESP32C3_AP_IDLE) {
            return;
        }
#if ESP32C3_WIFI_AP_DEBUG
        info_report("[ESP32-C3][AP] association request, accepting");
#endif
        s->ap_state = ESP32C3_AP_ASSOCIATED;
        esp32c3_wifi_ap_assoc_resp(s, sta);

        /*
         * Now there is somewhere for a data frame to go.  The delay is for the
         * station's own state machine: the association response has to reach
         * net80211 and be acted on before a frame belonging to the new BSS can
         * be recognised as belonging to it.
         */
        s->injected = 0;
        if (s->inject_frames != 0) {
            timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                      + (int64_t) s->inject_period_ms * SCALE_MS);
        }

        /* And then give up on it; see esp32c3_wifi_ap_deauth(). */
        if (s->rsn) {
            timer_mod(s->deauth_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                      + (int64_t) (s->inject_frames + 2)
                        * s->inject_period_ms * SCALE_MS);
        }
        break;

    case IEEE80211_SUBTYPE_DEAUTH:
    case IEEE80211_SUBTYPE_DISASSOC:
#if ESP32C3_WIFI_AP_DEBUG
        info_report("[ESP32-C3][AP] the station left");
#endif
        s->ap_state = ESP32C3_AP_IDLE;
        break;

    default:
        break;
    }
}

/*
 * The station handed a queue a frame and set the valid and enable bits.  The
 * descriptor is the same lldesc_t the receive side uses and its address is
 * split the same way, twenty bits in the queue's own register and the top
 * twelve in A_WIFI_DSCR_HIGH.
 */
static void esp32c3_wifi_tx(ESP32C3WifiState *s, unsigned q, uint32_t plcp0)
{
    uint32_t high = s->mem[A_WIFI_DSCR_HIGH / sizeof(uint32_t)] & 0xfff00000;
    uint32_t dscr_addr = high | (plcp0 & WIFI_TX_PLCP0_ADDR);
    uint32_t dscr[3];
    uint8_t  frame[1600];
    uint32_t len;

    address_space_read(&address_space_memory, dscr_addr, MEMTXATTRS_UNSPECIFIED,
                       dscr, sizeof(dscr));
    len = (le32_to_cpu(dscr[0]) & WIFI_DSCR_LENGTH_MASK)
          >> WIFI_DSCR_LENGTH_SHIFT;

    /*
     * The length counts the four bytes of frame check sequence the radio would
     * have appended, exactly as the receive side's sig_len does.  They are not
     * part of the frame.
     */
    if (len > 4 && len - 4 <= sizeof(frame)) {
        len -= 4;
        address_space_read(&address_space_memory, le32_to_cpu(dscr[1]),
                           MEMTXATTRS_UNSPECIFIED, frame, len);
#if ESP32C3_WIFI_AP_DEBUG
        info_report("[ESP32-C3][MAC] tx q%u %u bytes, frame control 0x%04x",
                    q, len, lduw_le_p(frame));
#endif
        esp32c3_wifi_ap_input(s, frame, len);
    }

    /*
     * Retire the descriptor as the DMA does, and say which queue it was.
     * lmacPostTxComplete reads the bitmap through hal_mac_get_txq_state() and
     * retires the entry through hal_mac_clr_txq_state(); with the bitmap left
     * at zero it finds nothing to complete, the queue never becomes free
     * again, and the station transmits exactly one frame for the whole run.
     */
    dscr[0] = cpu_to_le32(le32_to_cpu(dscr[0]) & ~WIFI_DSCR_OWNER_HW);
    address_space_write(&address_space_memory, dscr_addr, MEMTXATTRS_UNSPECIFIED,
                        dscr, sizeof(dscr[0]));

    s->txq_state |= 1u << q;
    s->int_status |= WIFI_INT_TX_COMPLETE;
    esp32c3_wifi_irq_update(s);
}

static uint64_t esp32c3_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(opaque);
    uint32_t r = s->mem[addr / sizeof(uint32_t)];

    switch (addr) {
    case A_WIFI_MAC_BRINGUP:
        /*
         * hal_init() sets bit 1 here and then spins until bit 0 reads set,
         * before it touches anything else in the block: a request/acknowledge
         * pair for MAC bring-up.  Bit 0 is the acknowledge and nothing in the
         * guest ever writes it.
         */
        r |= WIFI_MAC_BRINGUP_DONE;
        break;

    case A_WIFI_MAC_NOW:
        /*
         * The MAC's free-running microsecond counter.  hal_now() is a bare
         * load of it and esp_wifi_internal_get_mac_clock_time() returns it
         * unchanged, and from there it reaches rate control, the beacon and
         * scan timers, the AMPDU ageing and every "have we waited long
         * enough" test in net80211 -- 127 reads in one esp_wifi_start(), and
         * not one write.  Held as storage it reads zero forever, which is not
         * a hang but is worse than one: no elapsed time ever passes, so every
         * one of those tests silently takes the wrong branch.
         */
        r = (uint32_t) (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000);
        break;

    case A_WIFI_MAC_INT_STATUS:
        /*
         * What wDev_ProcessFiq dispatches on.  Storage is wrong here in the
         * direction that matters: it would read back whatever was last
         * written to A_WIFI_MAC_INT_CLR, so the handler would either see
         * nothing or see every bit it had just acknowledged.
         */
        r = s->int_status;
        break;

    case A_WIFI_TXQ_STATE:
        /*
         * Which queues have finished, for the same reason.  The clear is a
         * separate register below this one, so storage here would answer with
         * whatever was last retired rather than with what is outstanding.
         */
        r = s->txq_state;
        break;

    default:
        break;
    }

#if ESP32C3_WIFI_DEBUG
    info_report("[ESP32-C3][MAC] read  0x%04" HWADDR_PRIx " -> 0x%08x", addr, r);
#endif
    return r;
}

static void esp32c3_wifi_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned int size)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(opaque);

#if ESP32C3_WIFI_DEBUG
    info_report("[ESP32-C3][MAC] write 0x%04" HWADDR_PRIx " = 0x%08x", addr,
                (uint32_t) value);
#endif

    switch (addr) {
    case A_WIFI_MAC_INT_CLR:
        /*
         * Write one to clear.  It has to fall through to the store as well:
         * hal_mac_interrupt_clr_watchdog() reads this register back, sets a
         * bit and writes the result, so a register that read zero here would
         * acknowledge every event the handler had not looked at yet.
         */
        s->int_status &= ~(uint32_t) value;
        esp32c3_wifi_irq_update(s);
        break;

    case A_WIFI_RX_DSCR_RELOAD:
        /*
         * Another request/acknowledge pair, and the same trap as the bring-up
         * bit at 0x0d14: wDev_AppendRxBlocks() hands buffers back to the MAC,
         * sets bit 0 through hal_mac_rx_set_dscr_reload() and then spins on
         * hal_mac_rx_is_dscr_reload() until the MAC clears it.  Held as
         * storage it reads back set forever and the WiFi task never returns
         * from the first frame it was given.  The reload itself is a no-op
         * here: the list the MAC walks is the one in guest memory, and the
         * guest has just relinked it.
         */
        value &= ~(uint64_t) WIFI_RX_DSCR_RELOAD;
        break;

    case A_WIFI_RX_DSCR_BASE:
        /*
         * hal_mac_rx_set_base(): the head of the receive ring, and the last
         * thing the libraries do before they are ready for a frame.  Nothing
         * is delivered before this, so the access point starts beaconing here
         * rather than at reset, when there would be no descriptor to write a
         * beacon into.
         */
        s->rx_dscr_next = (uint32_t) value;

        if (value != 0) {
            timer_mod(s->beacon_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                      + 100 * SCALE_MS);
        }
        break;

    case A_WIFI_TXQ_STATE_CLR:
        /*
         * hal_mac_clr_txq_state( 2, mask ): retire the queues in the mask.
         * The accessor reads this register, ors the mask in and writes it
         * back, so what arrives is the whole set to retire rather than only
         * the queue newly added to it.
         */
        s->txq_state &= ~((uint32_t) value & WIFI_TXQ_STATE_MASK);
        break;

    default:
        break;
    }

    {
        unsigned q;

        for (q = 0; q < ESP32C3_WIFI_TX_QUEUES; q++) {
            if (addr != A_WIFI_TX_PLCP0(q)) {
                continue;
            }
            /*
             * hal_mac_txq_enable() sets both bits at once and is the only
             * thing that does; the valid bit is the one the MAC clears when
             * the frame has gone, and hal_mac_is_txq_valid() is what the
             * libraries poll.  Held as storage it never clears.
             */
            if ((value & (WIFI_TX_PLCP0_VALID | WIFI_TX_PLCP0_ENABLE))
                == (WIFI_TX_PLCP0_VALID | WIFI_TX_PLCP0_ENABLE)) {
                value &= ~(uint64_t) WIFI_TX_PLCP0_VALID;
                s->mem[addr / sizeof(uint32_t)] = (uint32_t) value;
                esp32c3_wifi_tx(s, q, (uint32_t) value);
                return;
            }
        }
    }

    s->mem[addr / sizeof(uint32_t)] = (uint32_t) value;
}

static const MemoryRegionOps esp32c3_wifi_ops = {
    .read =  esp32c3_wifi_read,
    .write = esp32c3_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32c3_wifi_reset_hold(Object *obj, ResetType type)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(obj);

    memset(s->mem, 0, sizeof(s->mem));
    s->int_status = 0;
    s->txq_state = 0;
    s->rx_dscr_next = 0;
    s->injected = 0;
    s->ap_state = ESP32C3_AP_IDLE;
    s->sta_channel = 0;
    s->seq = 0;

    while (s->rx_head != NULL) {
        ESP32C3WifiFrame *f = s->rx_head;

        s->rx_head = f->next;
        g_free(f);
    }
    s->rx_tail = NULL;

    timer_del(s->inject_timer);
    timer_del(s->beacon_timer);
    timer_del(s->deauth_timer);
    timer_del(s->rx_timer);
    esp32c3_wifi_irq_update(s);
}

static void esp32c3_wifi_init(Object *obj)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    static const uint8_t bssid[6] = { 0x02, 0x00, 0x00, 0xc3, 0x00, 0x01 };

    memory_region_init_io(&s->iomem, obj, &esp32c3_wifi_ops, s,
                          TYPE_ESP32C3_WIFI, ESP32C3_WIFI_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    memcpy(s->bssid, bssid, sizeof(s->bssid));

    s->inject_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32c3_wifi_inject, s);
    s->beacon_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32c3_wifi_ap_beacon,
                                   s);
    s->deauth_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32c3_wifi_ap_deauth,
                                   s);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32c3_wifi_ap_pump, s);
}

static Property esp32c3_wifi_properties[] = {
    DEFINE_PROP_STRING("ap-ssid", ESP32C3WifiState, ssid),
    DEFINE_PROP_UINT32("ap-channel", ESP32C3WifiState, channel, 1),
    DEFINE_PROP_BOOL("ap-privacy", ESP32C3WifiState, privacy, true),
    DEFINE_PROP_BOOL("ap-rsn", ESP32C3WifiState, rsn, true),
    DEFINE_PROP_UINT32("inject-frames", ESP32C3WifiState, inject_frames, 4),
    DEFINE_PROP_UINT32("inject-period-ms", ESP32C3WifiState, inject_period_ms,
                       200),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32c3_wifi_realize(DeviceState *dev, Error **errp)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(dev);

    /*
     * The default is what examples/wifi-net asks for, because an access point
     * with a different name is one the station will scan past: there is no
     * other way for the two to agree, the image being prebuilt.
     */
    if (s->ssid == NULL) {
        s->ssid = g_strdup("rtems-test-network");
    }
}

static void esp32c3_wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32c3_wifi_reset_hold;
    dc->realize = esp32c3_wifi_realize;
    device_class_set_props(dc, esp32c3_wifi_properties);
}

static const TypeInfo esp32c3_wifi_info = {
    .name = TYPE_ESP32C3_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32C3WifiState),
    .instance_init = esp32c3_wifi_init,
    .class_init = esp32c3_wifi_class_init
};

static void esp32c3_wifi_types(void)
{
    type_register_static(&esp32c3_wifi_info);
}

type_init(esp32c3_wifi_types)
