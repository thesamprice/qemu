/*
 * ESP32-C3 RF front-end (FE) register block emulation
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

#define TYPE_ESP32C3_FE "misc.esp32c3.fe"
#define ESP32C3_FE(obj) OBJECT_CHECK(ESP32C3FeState, (obj), TYPE_ESP32C3_FE)

#define ESP32C3_FE_REGS_SIZE        0x1000

/* IQ mismatch estimator status, see the comment in esp32c3_fe.c */
#define A_FE_IQ_EST_STATUS          0x174
#define FE_IQ_EST_DONE              BIT(16)

typedef struct ESP32C3FeState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    uint32_t mem[ESP32C3_FE_REGS_SIZE / sizeof(uint32_t)];
} ESP32C3FeState;
