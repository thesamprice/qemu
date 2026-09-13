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
 * hundreds of lines and this is three. */
#define ESP32C3_WIFI_RX_DEBUG   1

static void esp32c3_wifi_irq_update(ESP32C3WifiState *s)
{
    qemu_set_irq(s->irq, s->int_status != 0);
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
    uint32_t dscr_addr = s->rx_dscr_next;
    uint32_t dscr[3];
    uint32_t flags, size, total;
    uint8_t  hdr[WIFI_RX_CTRL_LEN];

    /* Before hal_mac_rx_set_base() runs there is no list to deliver into. */
    if (dscr_addr == 0) {
        return false;
    }

    address_space_read(&address_space_memory, dscr_addr, MEMTXATTRS_UNSPECIFIED,
                       dscr, sizeof(dscr));
    flags = le32_to_cpu(dscr[0]);
    size  = flags & WIFI_DSCR_SIZE_MASK;
    total = WIFI_RX_CTRL_LEN + len;

    if (le32_to_cpu(dscr[1]) == 0 || total > size) {
        return false;
    }

    /*
     * wifi_pkt_rx_ctrl_t, built by hand because only six of its fields are
     * read on this path and the rest are reserved even in the public header.
     * The two match bits say which interface address the frame was addressed
     * to; with neither set wDev_ProcessRxSucData reaches wDev_DiscardFrame
     * before it reaches net80211, which looks exactly like a frame that never
     * arrived.
     */
    memset(hdr, 0, sizeof(hdr));
    stl_le_p(hdr + 0,  ((uint32_t) (uint8_t) -60)              /* rssi, dBm */
                       | (11u << 8)                            /* rate      */
                       | WIFI_RX_CTRL_STA_MATCH);
    hdr[10] = 1;                                               /* channel   */
    stl_le_p(hdr + 12, (uint32_t) (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000));
    hdr[20] = (uint8_t) -97;                                   /* noise     */
    stl_le_p(hdr + 44, (uint32_t) len);                        /* sig_len,
                                                                  rx_state 0 */

    address_space_write(&address_space_memory, le32_to_cpu(dscr[1]),
                        MEMTXATTRS_UNSPECIFIED, hdr, sizeof(hdr));
    address_space_write(&address_space_memory,
                        le32_to_cpu(dscr[1]) + WIFI_RX_CTRL_LEN,
                        MEMTXATTRS_UNSPECIFIED, frame, len);

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
 * A frame to deliver.  There is no netdev behind this device yet, and until
 * there is one has to be made up: what has never happened is that the guest's
 * receive path ran at all, and a synthetic frame answers that as well as a
 * real one would.  An 802.11 data frame from the distribution system carrying
 * LLC/SNAP and an ARP request, addressed to whichever station address the
 * guest programmed into the MAC, because a frame addressed to anything else
 * is dropped in wDev_ProcessRxSucData and never reaches net80211 at all.
 *
 * As far as net80211 it does get.  It stops in sta_input(), which is correct
 * of it: the station has not associated -- nothing in the image calls
 * esp_wifi_connect() and there is no access point to call it about -- so
 * there is no BSS this frame could have come from.  Getting past that needs
 * the simulated access point esp32-open-mac's fork has, and a guest that asks
 * to associate with it.
 */
static size_t esp32c3_wifi_make_frame(ESP32C3WifiState *s, uint8_t *buf,
                                      size_t buflen)
{
    static const uint8_t ap[6] = { 0x02, 0x00, 0x00, 0xc3, 0x00, 0x01 };
    uint32_t lo = s->mem[A_WIFI_STA_ADDR_LOW / sizeof(uint32_t)];
    uint32_t hi = s->mem[A_WIFI_STA_ADDR_HIGH / sizeof(uint32_t)];
    uint8_t  sta[6];
    uint8_t *p = buf;

    assert(buflen >= 24 + 8 + 28);

    stl_le_p(sta, lo);
    stw_le_p(sta + 4, (uint16_t) hi);

    *p++ = 0x08;                        /* data, subtype 0        */
    *p++ = 0x02;                        /* from DS                */
    *p++ = 0x00; *p++ = 0x00;           /* duration               */
    memcpy(p, sta, 6);   p += 6;        /* addr1: receiver        */
    memcpy(p, ap,  6);   p += 6;        /* addr2: transmitter     */
    memcpy(p, ap,  6);   p += 6;        /* addr3: source          */
    *p++ = 0x00; *p++ = 0x00;           /* sequence control       */

    *p++ = 0xaa; *p++ = 0xaa; *p++ = 0x03;              /* LLC    */
    *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;              /* OUI    */
    *p++ = 0x08; *p++ = 0x06;                           /* ARP    */

    *p++ = 0x00; *p++ = 0x01;                           /* Ethernet */
    *p++ = 0x08; *p++ = 0x00;                           /* IPv4     */
    *p++ = 0x06; *p++ = 0x04;
    *p++ = 0x00; *p++ = 0x01;                           /* request  */
    memcpy(p, ap, 6);    p += 6;                        /* sender HW */
    *p++ = 192; *p++ = 168; *p++ = 4; *p++ = 1;         /* sender IP */
    memset(p, 0, 6);     p += 6;                        /* target HW */
    *p++ = 192; *p++ = 168; *p++ = 4; *p++ = 2;         /* target IP */

    return p - buf;
}

/*
 * Deliver on a timer rather than on host traffic.  A netdev is the right
 * answer and is what esp32-open-mac's fork does, but it cannot be the first
 * step here: the station never associates, so nothing the host sent would be
 * carried anyway, and the question this has to answer first is whether the
 * interrupt and the descriptor handover work at all.  Set inject-frames to 0
 * to turn it off.
 */
static void esp32c3_wifi_inject(void *opaque)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(opaque);
    uint8_t frame[128];
    size_t  len = esp32c3_wifi_make_frame(s, frame, sizeof(frame));

    if (esp32c3_wifi_rx_deliver(s, frame, len)) {
        s->injected++;
    }

    if (s->injected < s->inject_frames) {
        timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                  + (int64_t) s->inject_period_ms * SCALE_MS);
    }
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
         * thing the libraries do before they are ready for a frame.  Arming
         * the injection here rather than at reset is what keeps it from
         * writing into memory that is not a descriptor yet.
         */
        s->rx_dscr_next = (uint32_t) value;
        s->injected = 0;

        if (s->inject_frames != 0 && value != 0) {
            timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                      + (int64_t) s->inject_period_ms * SCALE_MS);
        }
        break;

    default:
        break;
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
    s->rx_dscr_next = 0;
    s->injected = 0;
    timer_del(s->inject_timer);
    esp32c3_wifi_irq_update(s);
}

static void esp32c3_wifi_init(Object *obj)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_wifi_ops, s,
                          TYPE_ESP32C3_WIFI, ESP32C3_WIFI_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->inject_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32c3_wifi_inject, s);
}

static Property esp32c3_wifi_properties[] = {
    DEFINE_PROP_UINT32("inject-frames", ESP32C3WifiState, inject_frames, 4),
    DEFINE_PROP_UINT32("inject-period-ms", ESP32C3WifiState, inject_period_ms,
                       200),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32c3_wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32c3_wifi_reset_hold;
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
