/*
 * ESP32-C3 WiFi MAC register block emulation
 *
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "qemu/timer.h"

#define TYPE_ESP32C3_WIFI "misc.esp32c3.wifi"
#define ESP32C3_WIFI(obj) OBJECT_CHECK(ESP32C3WifiState, (obj), TYPE_ESP32C3_WIFI)

/*
 * The MAC occupies three consecutive 4 KB pages and the HAL addresses all
 * three from the same base, so they are one window here rather than three
 * devices.  reg_base.h names none of them.
 */
#define DR_REG_WIFI_MAC_BASE        0x60033000
#define ESP32C3_WIFI_REGS_SIZE      0x3000

/* See the comments on each case in esp32c3_wifi.c. */
#define A_WIFI_MAC_BRINGUP          0x0d14
#define WIFI_MAC_BRINGUP_DONE       BIT(0)

#define A_WIFI_MAC_NOW              0x2000

/*
 * The receive path.  Each of these offsets is the operand of a one- or
 * two-instruction accessor in the image's .wifi_iram; the name in the comment
 * is that accessor, so each is checkable against the guest rather than
 * guessed.
 */
#define A_WIFI_RX_DSCR_RELOAD       0x0084  /* hal_mac_rx_set_dscr_reload    */
#define WIFI_RX_DSCR_RELOAD         BIT(0)  /* hal_mac_rx_is_dscr_reload     */
#define A_WIFI_RX_DSCR_BASE         0x0088  /* hal_mac_rx_set_base           */
#define A_WIFI_RX_DSCR_NEXT         0x008c  /* hal_mac_rx_read_rxdscrnext    */
#define A_WIFI_RX_DSCR_LAST         0x0090  /* hal_mac_rx_read_rxdscrlast    */
#define A_WIFI_MAC_INT_STATUS       0x0c3c  /* hal_mac_interrupt_get_event   */
#define A_WIFI_MAC_INT_CLR          0x0c40  /* hal_mac_interrupt_clr_event   */
#define A_WIFI_DSCR_HIGH            0x0c64  /* the top 12 bits of the above  */

/*
 * The transmit path, found the same way.  hal_mac_txq_enable() is five
 * instructions and computes its register as (0x0c0067a1 - q) << 3, which is
 * 0x60033d08 - 8q; hal_mac_is_txq_enabled() reads bit 31 of it and
 * hal_mac_is_txq_valid() bit 30, and hal_mac_set_txq_invalid() clears bit 30
 * alone.  The low twenty bits are the address of the descriptor list, in the
 * same split as the receive side: the top twelve come from A_WIFI_DSCR_HIGH.
 * hal_mac_tx_config_edca() puts the queue's parameters in the word below.
 */
#define ESP32C3_WIFI_TX_QUEUES      5
#define A_WIFI_TX_PLCP0(q)          (0x0d08 - 8 * (q))
#define WIFI_TX_PLCP0_ADDR          0x000fffff
#define WIFI_TX_PLCP0_VALID         BIT(30)
#define WIFI_TX_PLCP0_ENABLE        BIT(31)

/*
 * Which queue the transmit-complete interrupt was about.
 *
 * hal_mac_get_txq_state() and hal_mac_clr_txq_state() each take a kind and
 * have three arms, at three pairs of registers.  It is kind 2 that
 * lmacPostTxComplete reads -- 0x0cb0 is the only one of the six the guest
 * touches after a transmit -- and it is a bitmap of four queues, cleared by
 * writing the bits into the separate register below it rather than back into
 * itself.  So the two are separate here as they are in silicon: a
 * write-one-to-clear on one word would answer reads with what was last
 * retired instead of with what is outstanding.
 */
#define A_WIFI_TXQ_STATE_CLR        0x0cac  /* hal_mac_clr_txq_state( 2, .. ) */
#define A_WIFI_TXQ_STATE            0x0cb0  /* hal_mac_get_txq_state( 2 )     */
#define WIFI_TXQ_STATE_MASK         0x0000000f

/*
 * The event bits, as wDev_ProcessFiq tests them: it masks the word from
 * hal_mac_interrupt_get_event() against each in turn and calls the matching
 * handler.  RX_SUC_DATA is the one that reaches lmacProcessRxSucData.
 */
#define WIFI_INT_RX_SUC_DATA        0x01004000
#define WIFI_INT_TX_COMPLETE        0x00000080

/*
 * The DMA descriptor, which the MAC shares with every other Espressif
 * peripheral that takes a linked list -- ESP-IDF calls it lldesc_t.  Only the
 * first word is bitfields; the other two are the buffer and the next link.
 */
#define WIFI_DSCR_SIZE_MASK         0x00000fff
#define WIFI_DSCR_LENGTH_SHIFT      12
#define WIFI_DSCR_LENGTH_MASK       0x00fff000
#define WIFI_DSCR_EOF               BIT(30)
#define WIFI_DSCR_OWNER_HW          BIT(31)

/*
 * The metadata header the MAC writes in front of every received frame: it is
 * wifi_pkt_rx_ctrl_t from esp_wifi_types_native.h, 48 bytes on the C3, and
 * wDev_ProcessRxSucData expects the 802.11 frame to start immediately after
 * it.  The two match bits are in what the public header calls reserved.
 */
#define WIFI_RX_CTRL_LEN            48
#define WIFI_RX_CTRL_STA_MATCH      BIT(28)
#define WIFI_RX_CTRL_AP_MATCH       BIT(29)

/*
 * The station's own address, low four bytes then high two.  No accessor names
 * these; they are from the trace, where the pair reads back exactly what
 * esp_wifi_get_mac() reported, and 0x48 holds the same address plus one for
 * the soft-AP interface.  esp32-open-mac's ESP32 model reads its two
 * addresses from the same two offsets.
 */
#define A_WIFI_STA_ADDR_LOW         0x0040
#define A_WIFI_STA_ADDR_HIGH        0x0044

/*
 * The simulated access point.  It gets as far as association and no further:
 * the network it advertises is WPA2-PSK, because this image will not look at
 * an open one, and the four-way handshake behind that is not modelled.
 */
typedef enum {
    ESP32C3_AP_IDLE,
    ESP32C3_AP_AUTHENTICATED,
    ESP32C3_AP_ASSOCIATED
} ESP32C3WifiApState;

/* One frame waiting to be handed to the station, on the queue below. */
typedef struct ESP32C3WifiFrame {
    struct ESP32C3WifiFrame *next;
    uint16_t len;
    uint8_t  data[1600];
} ESP32C3WifiFrame;

typedef struct ESP32C3WifiState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t mem[ESP32C3_WIFI_REGS_SIZE / sizeof(uint32_t)];

    /* The MAC-driven half of the interrupt register, the queue-complete
     * bitmap, and the descriptor the next frame is written into.  None of the
     * three is storage. */
    uint32_t int_status;
    uint32_t txq_state;
    uint32_t rx_dscr_next;

    /* The access point.  ssid and channel are properties. */
    char              *ssid;
    uint32_t           channel;
    bool               privacy;
    bool               rsn;
    uint8_t            bssid[6];
    ESP32C3WifiApState ap_state;
    uint32_t           sta_channel;   /* where the station last said it was */
    uint16_t           seq;
    QEMUTimer         *beacon_timer;
    QEMUTimer         *deauth_timer;

    /* Frames the access point owes the station, oldest first. */
    ESP32C3WifiFrame  *rx_head;
    ESP32C3WifiFrame  *rx_tail;
    QEMUTimer         *rx_timer;

    /* See esp32c3_wifi_inject(); the first two are properties. */
    uint32_t   inject_frames;
    uint32_t   inject_period_ms;
    uint32_t   injected;
    QEMUTimer *inject_timer;
} ESP32C3WifiState;
