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

typedef struct ESP32C3WifiState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    uint32_t mem[ESP32C3_WIFI_REGS_SIZE / sizeof(uint32_t)];
} ESP32C3WifiState;
