/*
 * ESP32-C3 GPSPI2 controller
 *
 * The general purpose SPI peripheral, at DR_REG_SPI2_BASE.  Not the SPI
 * Memory controller: hw/ssi/esp32c3_spi.c models that one, its registers are
 * SPI_MEM_* and its command register holds flash opcodes, and the two are
 * different peripherals rather than variants of one design.
 *
 * The layout here is the same family as the original ESP32's unified SPI
 * peripheral -- CMD.USR starts a transfer, USER/USER1/USER2 describe the
 * phases, W0..W15 are the buffer -- renumbered for the C series.  Every offset
 * and field position below is from esp-idf v5.3.1
 * components/soc/esp32c3/include/soc/spi_reg.h rather than from the ESP32
 * model, because two of them moved: SPI_USR is bit 24 and not 18, and
 * USR_ADDR_BITLEN starts at 27 and not 26.
 *
 * Copyright (c) 2026 Samuel Price
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/ssi/ssi.h"

#define TYPE_ESP32C3_GPSPI "ssi.esp32c3.gpspi"
#define ESP32C3_GPSPI(obj) OBJECT_CHECK(Esp32C3GpspiState, (obj), TYPE_ESP32C3_GPSPI)

/* CS0..CS2 reach pads on this part. */
#define ESP32C3_GPSPI_CS_COUNT   3
/* W0 at 0x98 through W15 at 0xd4. */
#define ESP32C3_GPSPI_BUF_WORDS  16

typedef struct Esp32C3GpspiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq cs_gpio[ESP32C3_GPSPI_CS_COUNT];
    SSIBus *spi;

    uint32_t addr_reg;
    uint32_t ctrl_reg;
    uint32_t clock_reg;
    uint32_t user_reg;
    uint32_t user1_reg;
    uint32_t user2_reg;
    uint32_t ms_dlen_reg;
    uint32_t misc_reg;
    uint32_t dma_conf_reg;
    uint32_t slave_reg;
    uint32_t data_reg[ESP32C3_GPSPI_BUF_WORDS];
} Esp32C3GpspiState;

REG32(GPSPI_CMD, 0x00)
    /* Bit 24 on the C series.  The ESP32 has it at 18; using that value
     * silently never starts a transfer. */
    FIELD(GPSPI_CMD, USR, 24, 1)
    FIELD(GPSPI_CMD, UPDATE, 23, 1)
    FIELD(GPSPI_CMD, CONF_BITLEN, 0, 18)

REG32(GPSPI_ADDR, 0x04)
REG32(GPSPI_CTRL, 0x08)
REG32(GPSPI_CLOCK, 0x0c)

REG32(GPSPI_USER, 0x10)
    FIELD(GPSPI_USER, COMMAND, 31, 1)
    FIELD(GPSPI_USER, ADDR, 30, 1)
    FIELD(GPSPI_USER, DUMMY, 29, 1)
    FIELD(GPSPI_USER, MISO, 28, 1)
    FIELD(GPSPI_USER, MOSI, 27, 1)
    FIELD(GPSPI_USER, DOUTDIN, 0, 1)

REG32(GPSPI_USER1, 0x14)
    FIELD(GPSPI_USER1, ADDR_BITLEN, 27, 5)
    FIELD(GPSPI_USER1, DUMMY_CYCLELEN, 0, 8)

REG32(GPSPI_USER2, 0x18)
    FIELD(GPSPI_USER2, COMMAND_BITLEN, 28, 4)
    FIELD(GPSPI_USER2, COMMAND_VALUE, 0, 16)

/* One length register for both directions, where the ESP32 has separate
 * MOSI_DLEN and MISO_DLEN.  Holds bits minus one. */
REG32(GPSPI_MS_DLEN, 0x1c)
    FIELD(GPSPI_MS_DLEN, DATA_BITLEN, 0, 18)

REG32(GPSPI_MISC, 0x20)
    FIELD(GPSPI_MISC, CS0_DIS, 0, 1)
    FIELD(GPSPI_MISC, CS1_DIS, 1, 1)
    FIELD(GPSPI_MISC, CS2_DIS, 2, 1)

REG32(GPSPI_DMA_CONF, 0x30)
REG32(GPSPI_W0, 0x98)
REG32(GPSPI_W15, 0xd4)
REG32(GPSPI_SLAVE, 0xe0)
REG32(GPSPI_DATE, 0xf0)
