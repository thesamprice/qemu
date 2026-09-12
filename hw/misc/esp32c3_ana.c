/*
 * ESP32-C3 RF analog register bus (RTC I2C / analog master) emulation
 *
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

/*
 * DR_REG_RTC_I2C_BASE is the analog master: the only way the PHY can reach the
 * RF analog registers, which are not memory mapped at all.  A transaction is
 * one 32-bit write to I2C0_CTRL or I2C1_CTRL carrying the block ID, the
 * register index within that block, a data byte and a direction bit, then a
 * poll of BUSY in the same register; for a read, the byte appears in the DATA
 * field once BUSY drops.  The ROM spells it out at 0x40038e58
 * (rom_chip_i2c_readReg_org):
 *
 *   REG(0x44) = ~read_mask;                       // enable the block
 *   ctrl = 0x6000e000 + host_id * 4;
 *   *ctrl = EXEC | (reg_addr << 8) | block;       // EXEC, WRITE clear = read
 *   while (*ctrl & BUSY) { }
 *   return (*ctrl >> 16) & 0xff;
 *
 * and rom1_chip_i2c_writeReg in the image is the same with WRITE set and the
 * data byte in place.  So the bus is write-address/read-data and cannot be
 * answered by a constant: whatever libphy stores it later reads back, and the
 * calibration loops branch on what comes back.  Hence a backing store, one
 * byte per (block, index), rather than the fixed values the window used to
 * return.
 *
 * BUSY never reads set here, because a transaction retires inside the store
 * that started it.  That also makes rom1_i2c_master_reset() a no-op, which is
 * what we want -- it only pulses EXEC on a wedged channel.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32c3_ana.h"

/* Set to 1 to trace every access; the guest is silent when this block is
 * wrong, so this is usually the fastest way to find out what it asked for. */
#define ESP32C3_ANA_DEBUG   0

/*
 * Not every analog register is storage.  Some are status that the analog
 * block drives, and a store that only plays back writes reads them as zero
 * forever.  Each entry here is a bit that has to read as set; the store still
 * supplies the rest of the byte.
 *
 * They are recognisable in a trace because they are read and never written --
 * which is how this list was built, from the accesses of one run of
 * register_chipv7_phy().
 */
static const struct {
    uint8_t block;
    uint8_t reg;
    uint8_t set;
} esp32c3_ana_status[] = {
    /*
     * wait_rfpll_cal_end() polls bit 1 of register 7 of the PLL block for
     * "RF PLL calibration converged", 100 times at 20us, and on giving up
     * prints "error: pll_cal exceeds 2ms!!!" and carries on with an
     * uncalibrated PLL.  So this one is visible rather than fatal, and the
     * message is the only sign that the bit is not modelled.
     */
    { 0x62, 0x07, BIT(1) },
};

static uint64_t esp32c3_ana_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3AnaState *s = ESP32C3_ANA(opaque);
    uint32_t r = s->mem[addr / sizeof(uint32_t)];

    switch (addr) {
    case A_ANA_TXDC_CAL:
        /*
         * TX DC offset calibration.  txdc_cal_v70() rewrites the low 24 bits
         * with bit 1 toggled to start a comparison, then spins until bit 24
         * reports it finished; bits 31 and 30 are the comparator result it
         * feeds back into a 12-step binary search.  Bit 24 is status, so it
         * has to be driven rather than read back from what was written, or
         * the spin never ends.
         */
        r |= ANA_TXDC_CAL_DONE;
        break;

    case A_ANA_PKDET_STATUS:
        /*
         * ram_pkdet_vol_start() spins until bits 26:24 read 7, twice, once
         * either side of a toggle of bit 1.  Same thing: read-only status.
         */
        r |= ANA_PKDET_STATUS_MASK;
        break;

    default:
        break;
    }

#if ESP32C3_ANA_DEBUG
    info_report("[ESP32-C3][ANA] read  0x%03" HWADDR_PRIx " -> 0x%08x", addr, r);
#endif
    return r;
}

static void esp32c3_ana_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned int size)
{
    ESP32C3AnaState *s = ESP32C3_ANA(opaque);
    uint32_t v = (uint32_t) value;

    if (addr == A_ANA_I2C0_CTRL || addr == A_ANA_I2C1_CTRL) {
        if (FIELD_EX32(v, ANA_I2C_CTRL, EXEC)) {
            const uint8_t id = FIELD_EX32(v, ANA_I2C_CTRL, SLAVE_ID);
            const uint8_t reg = FIELD_EX32(v, ANA_I2C_CTRL, REG_ADDR);

            if (FIELD_EX32(v, ANA_I2C_CTRL, WRITE)) {
                s->block[id][reg] = FIELD_EX32(v, ANA_I2C_CTRL, DATA);
            } else {
                uint8_t data = s->block[id][reg];

                for (size_t i = 0; i < ARRAY_SIZE(esp32c3_ana_status); i++) {
                    if (esp32c3_ana_status[i].block == id &&
                        esp32c3_ana_status[i].reg == reg) {
                        data |= esp32c3_ana_status[i].set;
                    }
                }
                v = FIELD_DP32(v, ANA_I2C_CTRL, DATA, data);
            }
#if ESP32C3_ANA_DEBUG
            info_report("[ESP32-C3][ANA] i2c%d %s block 0x%02x reg 0x%02x = 0x%02x",
                        addr == A_ANA_I2C0_CTRL ? 0 : 1,
                        FIELD_EX32(v, ANA_I2C_CTRL, WRITE) ? "wr" : "rd",
                        id, reg, (unsigned) FIELD_EX32(v, ANA_I2C_CTRL, DATA));
#endif
        }

        /* The transaction is over by the time the guest can look, so it must
         * not find BUSY set, whatever it happened to write into that bit. */
        v = FIELD_DP32(v, ANA_I2C_CTRL, BUSY, 0);
    }
#if ESP32C3_ANA_DEBUG
    else {
        info_report("[ESP32-C3][ANA] write 0x%03" HWADDR_PRIx " = 0x%08x", addr, v);
    }
#endif

    s->mem[addr / sizeof(uint32_t)] = v;
}

static const MemoryRegionOps esp32c3_ana_ops = {
    .read =  esp32c3_ana_read,
    .write = esp32c3_ana_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32c3_ana_reset_hold(Object *obj, ResetType type)
{
    ESP32C3AnaState *s = ESP32C3_ANA(obj);

    memset(s->mem, 0, sizeof(s->mem));
    memset(s->block, 0, sizeof(s->block));
}

static void esp32c3_ana_init(Object *obj)
{
    ESP32C3AnaState *s = ESP32C3_ANA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_ana_ops, s,
                          TYPE_ESP32C3_ANA, ESP32C3_ANA_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32c3_ana_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32c3_ana_reset_hold;
}

static const TypeInfo esp32c3_ana_info = {
    .name = TYPE_ESP32C3_ANA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32C3AnaState),
    .instance_init = esp32c3_ana_init,
    .class_init = esp32c3_ana_class_init
};

static void esp32c3_ana_types(void)
{
    type_register_static(&esp32c3_ana_info);
}

type_init(esp32c3_ana_types)
