/*
 * ESP32-C3 RF analog register bus (RTC I2C / analog master) emulation
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

#define TYPE_ESP32C3_ANA "misc.esp32c3.ana"
#define ESP32C3_ANA(obj) OBJECT_CHECK(ESP32C3AnaState, (obj), TYPE_ESP32C3_ANA)

/*
 * The block occupies a whole peripheral slot.  Only the first 0x170 bytes are
 * known to be touched, but libphy reaches well past the 0x100 the TRM
 * documents, so there is no point guessing where it ends.
 */
#define ESP32C3_ANA_REGS_SIZE       0x1000

/* The two analog-master channels, selected by the block's host ID */
#define A_ANA_I2C0_CTRL             0x00
#define A_ANA_I2C1_CTRL             0x04

/*
 * Fields of the two control registers.  A write with EXEC set performs one
 * transaction against the addressed analog register and BUSY reads back set
 * until it retires.
 */
REG32(ANA_I2C_CTRL, 0x00)
    FIELD(ANA_I2C_CTRL, SLAVE_ID, 0, 8)
    FIELD(ANA_I2C_CTRL, REG_ADDR, 8, 8)
    FIELD(ANA_I2C_CTRL, DATA, 16, 8)
    FIELD(ANA_I2C_CTRL, WRITE, 24, 1)
    FIELD(ANA_I2C_CTRL, BUSY, 25, 1)
    FIELD(ANA_I2C_CTRL, EXEC, 26, 1)

/* Registers whose status bits are read-only and have to be driven, see below */
#define A_ANA_TXDC_CAL              0x4C
#define A_ANA_PKDET_STATUS          0x50

#define ANA_TXDC_CAL_DONE           BIT(24)
#define ANA_PKDET_STATUS_MASK       (7 << 24)

/* Address space reachable over the bus: an 8-bit block ID and an 8-bit index */
#define ESP32C3_ANA_BLOCK_COUNT     256
#define ESP32C3_ANA_BLOCK_REGS      256

typedef struct ESP32C3AnaState {
    SysBusDevice parent_object;
    MemoryRegion iomem;

    /* The directly addressed registers of the analog master itself */
    uint32_t mem[ESP32C3_ANA_REGS_SIZE / sizeof(uint32_t)];

    /* The analog registers behind it, which the master only reaches serially */
    uint8_t block[ESP32C3_ANA_BLOCK_COUNT][ESP32C3_ANA_BLOCK_REGS];
} ESP32C3AnaState;
