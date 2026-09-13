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

typedef struct ESP32C3WifiState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t mem[ESP32C3_WIFI_REGS_SIZE / sizeof(uint32_t)];

    /* The MAC-driven half of the interrupt register, and the descriptor the
     * next frame is written into.  Neither is storage. */
    uint32_t int_status;
    uint32_t rx_dscr_next;

    /* See esp32c3_wifi_inject(); the first two are properties. */
    uint32_t   inject_frames;
    uint32_t   inject_period_ms;
    uint32_t   injected;
    QEMUTimer *inject_timer;
} ESP32C3WifiState;
