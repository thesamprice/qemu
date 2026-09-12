/*
 * ESP32-C3 RF front-end (FE) register block emulation
 *
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

/*
 * DR_REG_FE_BASE is undocumented and libphy touches it from over 250 places,
 * so there is nothing to model register by register.  What it needs is the
 * same thing the analog bus needed: storage, so that a read returns the last
 * write instead of the zero the machine's catch-all I/O region used to
 * return, plus the few bits that are status rather than storage and have to
 * be driven or a poll of them never ends.
 *
 * esp32-open-mac's fork does exactly this for the ESP32 in hw/misc/esp32_fe.c,
 * with one forced offset, which is where the shape comes from.  The offsets
 * are not the same on the C3 and were found from the image, not copied.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32c3_fe.h"

#define ESP32C3_FE_DEBUG    0

static uint64_t esp32c3_fe_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3FeState *s = ESP32C3_FE(opaque);
    uint32_t r = s->mem[addr / sizeof(uint32_t)];

    switch (addr) {
    case A_FE_IQ_EST_STATUS:
        /*
         * ram_iq_est_enable() starts the IQ mismatch estimator and then spins
         * on bit 16 here for "estimate ready", counting RX gain samples from
         * the baseband meanwhile.  Nothing ever writes the bit, so storage
         * alone leaves that spin permanent.
         */
        r |= FE_IQ_EST_DONE;
        break;

    default:
        break;
    }

#if ESP32C3_FE_DEBUG
    info_report("[ESP32-C3][FE] read  0x%03" HWADDR_PRIx " -> 0x%08x", addr, r);
#endif
    return r;
}

static void esp32c3_fe_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size)
{
    ESP32C3FeState *s = ESP32C3_FE(opaque);

#if ESP32C3_FE_DEBUG
    info_report("[ESP32-C3][FE] write 0x%03" HWADDR_PRIx " = 0x%08x", addr,
                (uint32_t) value);
#endif
    s->mem[addr / sizeof(uint32_t)] = (uint32_t) value;
}

static const MemoryRegionOps esp32c3_fe_ops = {
    .read =  esp32c3_fe_read,
    .write = esp32c3_fe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32c3_fe_reset_hold(Object *obj, ResetType type)
{
    ESP32C3FeState *s = ESP32C3_FE(obj);

    memset(s->mem, 0, sizeof(s->mem));
}

static void esp32c3_fe_init(Object *obj)
{
    ESP32C3FeState *s = ESP32C3_FE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_fe_ops, s,
                          TYPE_ESP32C3_FE, ESP32C3_FE_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32c3_fe_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32c3_fe_reset_hold;
}

static const TypeInfo esp32c3_fe_info = {
    .name = TYPE_ESP32C3_FE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32C3FeState),
    .instance_init = esp32c3_fe_init,
    .class_init = esp32c3_fe_class_init
};

static void esp32c3_fe_types(void)
{
    type_register_static(&esp32c3_fe_info);
}

type_init(esp32c3_fe_types)
