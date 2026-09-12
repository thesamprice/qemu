/*
 * ESP32-C3 GPSPI2 controller
 *
 * Copyright (c) 2026 Samuel Price
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/esp32c3_gpspi.h"

/*
 * The transfer machinery is the ESP32 model's, which is the part worth
 * reusing; the register decode is not, because the C series renumbered it.
 *
 * What is gone entirely is the flash half.  The ESP32's peripheral serves both
 * the flash and general use, so its do_command() dispatches a dozen flash
 * opcodes.  GPSPI2 has no flash attached and only CMD.USR starts anything, so
 * this is the USR path and nothing else.
 */

typedef struct Esp32C3GpspiTransaction {
    uint32_t cmd;
    int      cmd_bytes;
    uint32_t addr;
    int      addr_bytes;
    void    *data;
    int      data_tx_bytes;
    int      data_rx_bytes;
} Esp32C3GpspiTransaction;

/* Convert one of the hardware "bitlen" registers to a byte count.  They hold
 * the number of bits minus one. */
static inline int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8;
}

static void esp32c3_gpspi_txrx_buffer(Esp32C3GpspiState *s, void *buf,
                                      int tx_bytes, int rx_bytes)
{
    int bytes = MAX(tx_bytes, rx_bytes);
    uint8_t *c_buf = (uint8_t *) buf;

    for (int i = 0; i < bytes; ++i) {
        uint8_t byte = 0;

        /* Both bounds are the loop index.  The ESP32 model compares the data
         * byte instead, which makes whether a received byte is kept depend on
         * the value of the byte just sent. */
        if (i < tx_bytes) {
            memcpy(&byte, c_buf + i, 1);
        }
        uint32_t res = ssi_transfer(s->spi, byte);
        if (i < rx_bytes) {
            memcpy(c_buf + i, &res, 1);
        }
    }
}

/*
 * MISC's CSn_DIS bits are "disable", so a zero bit means that chip select
 * takes part.  Asserting means driving low, hence the inversion.
 */
static void esp32c3_gpspi_cs_set(Esp32C3GpspiState *s, int value)
{
    for (int i = 0; i < ESP32C3_GPSPI_CS_COUNT; ++i) {
        bool disabled = (s->misc_reg & (1u << i)) != 0;
        qemu_set_irq(s->cs_gpio[i], disabled ? 1 : value);
    }
}

static void esp32c3_gpspi_transaction(Esp32C3GpspiState *s,
                                      Esp32C3GpspiTransaction *t)
{
    esp32c3_gpspi_cs_set(s, 0);
    esp32c3_gpspi_txrx_buffer(s, &t->cmd, t->cmd_bytes, 0);
    esp32c3_gpspi_txrx_buffer(s, &t->addr, t->addr_bytes, 0);
    esp32c3_gpspi_txrx_buffer(s, t->data, t->data_tx_bytes, t->data_rx_bytes);
    esp32c3_gpspi_cs_set(s, 1);
}

static void esp32c3_gpspi_do_transfer(Esp32C3GpspiState *s)
{
    Esp32C3GpspiTransaction t = { 0 };
    int data_bytes;

    if (FIELD_EX32(s->user_reg, GPSPI_USER, COMMAND)) {
        t.cmd = FIELD_EX32(s->user2_reg, GPSPI_USER2, COMMAND_VALUE);
        t.cmd_bytes =
            bitlen_to_bytes(FIELD_EX32(s->user2_reg, GPSPI_USER2, COMMAND_BITLEN));
    }

    if (FIELD_EX32(s->user_reg, GPSPI_USER, ADDR)) {
        t.addr_bytes =
            bitlen_to_bytes(FIELD_EX32(s->user1_reg, GPSPI_USER1, ADDR_BITLEN));
        /* The address is sent most significant byte first, and txrx_buffer
         * walks the word from its low address, so byte-swap and shift the
         * wanted bytes down. */
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
    }

    /* One length register serves both directions here, unlike the ESP32. */
    data_bytes = bitlen_to_bytes(FIELD_EX32(s->ms_dlen_reg, GPSPI_MS_DLEN,
                                            DATA_BITLEN));

    if (FIELD_EX32(s->user_reg, GPSPI_USER, MOSI)) {
        t.data = &s->data_reg[0];
        t.data_tx_bytes = data_bytes;
    }
    if (FIELD_EX32(s->user_reg, GPSPI_USER, MISO)) {
        t.data = &s->data_reg[0];
        t.data_rx_bytes = data_bytes;
    }

    esp32c3_gpspi_transaction(s, &t);
}

static uint64_t esp32c3_gpspi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32C3GpspiState *s = ESP32C3_GPSPI(opaque);

    if (addr >= A_GPSPI_W0 && addr <= A_GPSPI_W15) {
        return s->data_reg[(addr - A_GPSPI_W0) / sizeof(uint32_t)];
    }

    switch (addr) {
    case A_GPSPI_CMD:      return 0; /* USR self-clears; a transfer is instant */
    case A_GPSPI_ADDR:     return s->addr_reg;
    case A_GPSPI_CTRL:     return s->ctrl_reg;
    case A_GPSPI_CLOCK:    return s->clock_reg;
    case A_GPSPI_USER:     return s->user_reg;
    case A_GPSPI_USER1:    return s->user1_reg;
    case A_GPSPI_USER2:    return s->user2_reg;
    case A_GPSPI_MS_DLEN:  return s->ms_dlen_reg;
    case A_GPSPI_MISC:     return s->misc_reg;
    case A_GPSPI_DMA_CONF: return s->dma_conf_reg;
    case A_GPSPI_SLAVE:    return s->slave_reg;
    case A_GPSPI_DATE:     return 0x02107190; /* what the silicon reports */
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented read 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }
}

static void esp32c3_gpspi_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    Esp32C3GpspiState *s = ESP32C3_GPSPI(opaque);

    if (addr >= A_GPSPI_W0 && addr <= A_GPSPI_W15) {
        s->data_reg[(addr - A_GPSPI_W0) / sizeof(uint32_t)] = value;
        return;
    }

    switch (addr) {
    case A_GPSPI_CMD:
        /* UPDATE latches the configuration registers on real silicon.  Every
         * register here is read back from the same storage the guest wrote, so
         * there is nothing to latch and it is accepted and ignored. */
        if (FIELD_EX32(value, GPSPI_CMD, USR)) {
            esp32c3_gpspi_do_transfer(s);
        }
        break;
    case A_GPSPI_ADDR:     s->addr_reg = value; break;
    case A_GPSPI_CTRL:     s->ctrl_reg = value; break;
    case A_GPSPI_CLOCK:    s->clock_reg = value; break;
    case A_GPSPI_USER:     s->user_reg = value; break;
    case A_GPSPI_USER1:    s->user1_reg = value; break;
    case A_GPSPI_USER2:    s->user2_reg = value; break;
    case A_GPSPI_MS_DLEN:  s->ms_dlen_reg = value; break;
    case A_GPSPI_MISC:     s->misc_reg = value; break;
    case A_GPSPI_DMA_CONF: s->dma_conf_reg = value; break;
    case A_GPSPI_SLAVE:    s->slave_reg = value; break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", __func__, addr, value);
        break;
    }
}

static const MemoryRegionOps esp32c3_gpspi_ops = {
    .read = esp32c3_gpspi_read,
    .write = esp32c3_gpspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_gpspi_reset_hold(Object *obj, ResetType type)
{
    Esp32C3GpspiState *s = ESP32C3_GPSPI(obj);

    /*
     * Connect each chip select to whatever is on the bus at that index.
     *
     * This has to happen here rather than in realize because a peripheral
     * attached with -device does not exist yet when the machine is built, and
     * reset runs after it does.  hw/ssi/aspeed_smc.c does the same thing for
     * the same reason.
     *
     * Without it the slave never sees a chip select edge, and the symptom is
     * not silence: ssi_transfer_raw_default() treats a peripheral whose cs was
     * never driven as selected, so the *first* transaction works and every one
     * after it returns zeros, because the device is never returned to its idle
     * state between them.
     */
    for (int i = 0; i < ESP32C3_GPSPI_CS_COUNT; ++i) {
        DeviceState *dev = ssi_get_cs(s->spi, i);

        if (dev != NULL) {
            qdev_connect_gpio_out_named(DEVICE(s), SSI_GPIO_CS, i,
                                        qdev_get_gpio_in_named(dev,
                                                               SSI_GPIO_CS, 0));
        }
    }

    s->addr_reg = 0;
    s->ctrl_reg = 0;
    s->clock_reg = 0;
    s->user_reg = 0;
    s->user1_reg = 0;
    s->user2_reg = 0;
    s->ms_dlen_reg = 0;
    /* Every chip select disabled out of reset, which is what the silicon does
     * and what stops a stray transfer asserting one. */
    s->misc_reg = R_GPSPI_MISC_CS0_DIS_MASK | R_GPSPI_MISC_CS1_DIS_MASK
                | R_GPSPI_MISC_CS2_DIS_MASK;
    s->dma_conf_reg = 0;
    s->slave_reg = 0;
    memset(s->data_reg, 0, sizeof(s->data_reg));

    esp32c3_gpspi_cs_set(s, 1);
}

static void esp32c3_gpspi_realize(DeviceState *dev, Error **errp)
{
}

static void esp32c3_gpspi_init(Object *obj)
{
    Esp32C3GpspiState *s = ESP32C3_GPSPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_gpspi_ops, s,
                          TYPE_ESP32C3_GPSPI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    /* Named "gpspi2" rather than "spi", because the SPI Memory controller
     * already creates a bus called "spi" and -device bus= cannot tell two
     * identically named buses apart. */
    s->spi = ssi_create_bus(DEVICE(s), "gpspi2");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS,
                             ESP32C3_GPSPI_CS_COUNT);
}

static void esp32c3_gpspi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32c3_gpspi_realize;
    rc->phases.hold = esp32c3_gpspi_reset_hold;
}

static const TypeInfo esp32c3_gpspi_info = {
    .name          = TYPE_ESP32C3_GPSPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32C3GpspiState),
    .instance_init = esp32c3_gpspi_init,
    .class_init    = esp32c3_gpspi_class_init,
};

static void esp32c3_gpspi_register_types(void)
{
    type_register_static(&esp32c3_gpspi_info);
}

type_init(esp32c3_gpspi_register_types)
