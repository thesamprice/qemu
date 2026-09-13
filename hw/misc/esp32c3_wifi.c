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
 * esp32-open-mac's fork does the same for the ESP32 in hw/misc/esp32_wifi.c.
 * The offsets there are ESP32 offsets and do not carry over; these were found
 * from this image.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32c3_wifi.h"

/* Set to 1 to trace every access.  A wrong answer here produces no fault and
 * no unimplemented-register warning, so this is the only way to see what the
 * MAC was asked for; see the commit that added esp32c3_ana.c. */
#define ESP32C3_WIFI_DEBUG      0

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
}

static void esp32c3_wifi_init(Object *obj)
{
    ESP32C3WifiState *s = ESP32C3_WIFI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_wifi_ops, s,
                          TYPE_ESP32C3_WIFI, ESP32C3_WIFI_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32c3_wifi_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32c3_wifi_reset_hold;
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
