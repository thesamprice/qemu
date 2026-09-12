/*
 * ESP32-C3 GPIO emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/gpio/esp32c3_gpio.h"

/* Reset value of GPIO_DATE, which esp-idf reads to identify the peripheral. */
#define ESP32C3_GPIO_DATE_VALUE 0x2006130

static bool esp32c3_gpio_pin_index(hwaddr addr, hwaddr first, unsigned count,
                                   unsigned *index)
{
    if (addr < first || addr >= first + 4 * count) {
        return false;
    }
    *index = (addr - first) / 4;
    return true;
}

/*
 * The level an unconnected pad settles to.  A pull-up wins over a
 * pull-down only because something has to; asking for both is a
 * configuration error the hardware does not define either.
 */
static uint32_t esp32c3_gpio_pull_level(ESP32C3GPIOState *s)
{
    uint32_t level = 0;

    for (unsigned i = 0; i < ESP32C3_GPIO_PIN_COUNT; i++) {
        if (FIELD_EX32(s->iomux[i], IOMUX_PIN0, FUN_PU) &&
            !FIELD_EX32(s->iomux[i], IOMUX_PIN0, FUN_PD)) {
            level |= 1u << i;
        }
    }
    return level;
}

/* Status bits that a level-triggered pin asserts for as long as the level
 * holds.  Recomputed on every update rather than latched, so acknowledging
 * one through GPIO_STATUS_W1TC while the level persists re-raises it, as the
 * hardware does. */
static uint32_t esp32c3_gpio_level_status(ESP32C3GPIOState *s, uint32_t in)
{
    uint32_t status = 0;

    for (unsigned i = 0; i < ESP32C3_GPIO_PIN_COUNT; i++) {
        uint32_t bit = 1u << i;
        switch (FIELD_EX32(s->pin[i], GPIO_PIN0, INT_TYPE)) {
        case ESP32C3_GPIO_INTR_LOW_LEVEL:
            if (!(in & bit)) {
                status |= bit;
            }
            break;
        case ESP32C3_GPIO_INTR_HIGH_LEVEL:
            if (in & bit) {
                status |= bit;
            }
            break;
        default:
            break;
        }
    }
    return status;
}

/* Status bits an edge on one of the changed pins asserts. */
static uint32_t esp32c3_gpio_edge_status(ESP32C3GPIOState *s, uint32_t changed,
                                         uint32_t in)
{
    uint32_t status = 0;

    while (changed) {
        unsigned i = ctz32(changed);
        uint32_t bit = 1u << i;

        changed &= ~bit;
        if (i >= ESP32C3_GPIO_PIN_COUNT) {
            continue;
        }
        switch (FIELD_EX32(s->pin[i], GPIO_PIN0, INT_TYPE)) {
        case ESP32C3_GPIO_INTR_POSEDGE:
            if (in & bit) {
                status |= bit;
            }
            break;
        case ESP32C3_GPIO_INTR_NEGEDGE:
            if (!(in & bit)) {
                status |= bit;
            }
            break;
        case ESP32C3_GPIO_INTR_ANYEDGE:
            status |= bit;
            break;
        default:
            break;
        }
    }
    return status;
}

static uint32_t esp32c3_gpio_int_ena_mask(ESP32C3GPIOState *s, uint32_t which)
{
    uint32_t mask = 0;

    for (unsigned i = 0; i < ESP32C3_GPIO_PIN_COUNT; i++) {
        if (FIELD_EX32(s->pin[i], GPIO_PIN0, INT_ENA) & which) {
            mask |= 1u << i;
        }
    }
    return mask;
}

/*
 * Recompute the pad level and everything that follows from it.
 *
 * GPIO_IN reads the pad, not the output register: an enabled output drives
 * the pad and the input buffer reads back what is driven.  That is what lets
 * a driver test itself with no external wiring -- set a pin, read it back --
 * and it is also why an output toggling can raise this pin's own interrupt.
 */
/* The pads whose GPIO_PINn.PAD_DRIVER selects open drain. */
static uint32_t esp32c3_gpio_open_drain(ESP32C3GPIOState *s)
{
    uint32_t od = 0;

    for (unsigned i = 0; i < ESP32C3_GPIO_PIN_COUNT; ++i) {
        if (FIELD_EX32(s->pin[i], GPIO_PIN0, PAD_DRIVER)) {
            od |= 1u << i;
        }
    }
    return od;
}

static void esp32c3_gpio_update(ESP32C3GPIOState *s)
{
    /*
     * Open drain: the pad can pull low and cannot drive high.  So an enabled
     * output whose PAD_DRIVER is set only drives when its level is 0; writing
     * 1 releases the pad, and what the input then reads is whatever else is on
     * it -- a pull resistor, an external driver, or another pad on the same
     * net.  A push-pull output drives either way, which is the difference.
     */
    uint32_t od = esp32c3_gpio_open_drain(s);
    uint32_t driving = s->enable & ~(od & s->out) & ESP32C3_GPIO_PIN_MASK;
    uint32_t floating = ~driving & ESP32C3_GPIO_PIN_MASK;
    uint32_t pulls = esp32c3_gpio_pull_level(s);
    uint32_t in;
    uint32_t changed;

    in = (s->out & driving)
       | (s->ext_level & s->ext_valid & floating)
       | (pulls & ~s->ext_valid & floating);
    in &= ESP32C3_GPIO_PIN_MASK;

    /*
     * Pads tied together on one net, which is what "wire" names.  Real open
     * drain needs more than one driver on a line before it means anything --
     * the property being tested is that one holding low wins over another
     * releasing -- and two pads of one controller are not connected unless
     * something says they are.  A board says it with a track.
     *
     * The net is low if any pad on it is driving low, and otherwise takes the
     * pull level of the pads on it.  A net with two push-pull outputs
     * disagreeing is a short, and is left reading low rather than pretending
     * to resolve it.
     */
    if (s->wire != 0) {
        uint32_t net = s->wire & ESP32C3_GPIO_PIN_MASK;

        if ((driving & net & ~s->out) != 0) {
            in &= ~net;
        } else if ((pulls & net) != 0) {
            in |= net;
        } else {
            in &= ~net;
        }
    }

    changed = in ^ s->in;
    s->in = in;

    s->status |= esp32c3_gpio_edge_status(s, changed, in);
    s->status |= esp32c3_gpio_level_status(s, in);
    s->status &= ESP32C3_GPIO_PIN_MASK;

    while (changed) {
        unsigned i = ctz32(changed);
        changed &= ~(1u << i);
        if (s->output[i]) {
            qemu_set_irq(s->output[i], !!(in & (1u << i)));
        }
    }

    qemu_set_irq(s->parent.irq,
                 !!(s->status & esp32c3_gpio_int_ena_mask(s, ESP32C3_GPIO_INT_ENA_CPU)));
    qemu_set_irq(s->nmi_irq,
                 !!(s->status & esp32c3_gpio_int_ena_mask(s, ESP32C3_GPIO_INT_ENA_NMI)));
}

static uint64_t esp32c3_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3GPIOState *s = ESP32C3_GPIO(opaque);
    unsigned index;

    switch (addr) {
    case A_GPIO_BT_SELECT:
        return s->bt_select;
    case A_GPIO_OUT:
    case A_GPIO_OUT_W1TS:
    case A_GPIO_OUT_W1TC:
        /* The W1TS and W1TC aliases read back the register they modify. */
        return s->out;
    case A_GPIO_SDIO_SELECT:
        return s->sdio_select;
    case A_GPIO_ENABLE:
    case A_GPIO_ENABLE_W1TS:
    case A_GPIO_ENABLE_W1TC:
        return s->enable;
    case A_GPIO_STRAP:
        return s->parent.strap_mode;
    case A_GPIO_IN:
        return s->in;
    case A_GPIO_STATUS:
    case A_GPIO_STATUS_W1TS:
    case A_GPIO_STATUS_W1TC:
    case A_GPIO_STATUS_NEXT:
        return s->status;
    case A_GPIO_PCPU_INT:
        return s->status & esp32c3_gpio_int_ena_mask(s, ESP32C3_GPIO_INT_ENA_CPU);
    case A_GPIO_PCPU_NMI_INT:
        return s->status & esp32c3_gpio_int_ena_mask(s, ESP32C3_GPIO_INT_ENA_NMI);
    case A_GPIO_CPUSDIO_INT:
        return 0;
    case A_GPIO_CLOCK_GATE:
        return s->clock_gate;
    case A_GPIO_DATE:
        return ESP32C3_GPIO_DATE_VALUE;
    default:
        break;
    }

    if (esp32c3_gpio_pin_index(addr, A_GPIO_PIN0, ESP32C3_GPIO_PIN_COUNT, &index)) {
        return s->pin[index];
    }
    if (esp32c3_gpio_pin_index(addr, A_GPIO_FUNC0_IN_SEL_CFG,
                               ESP32C3_GPIO_FUNC_IN_COUNT, &index)) {
        return s->func_in_sel[index];
    }
    if (esp32c3_gpio_pin_index(addr, A_GPIO_FUNC0_OUT_SEL_CFG,
                               ESP32C3_GPIO_PIN_COUNT, &index)) {
        return s->func_out_sel[index];
    }

    qemu_log_mask(LOG_UNIMP, "%s: unimplemented register 0x%" HWADDR_PRIx "\n",
                  __func__, addr);
    return 0;
}

static void esp32c3_gpio_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned int size)
{
    ESP32C3GPIOState *s = ESP32C3_GPIO(opaque);
    uint32_t v = (uint32_t)value;
    unsigned index;

    switch (addr) {
    case A_GPIO_BT_SELECT:
        s->bt_select = v;
        return;
    case A_GPIO_OUT:
        s->out = v & ESP32C3_GPIO_PIN_MASK;
        break;
    case A_GPIO_OUT_W1TS:
        s->out |= v & ESP32C3_GPIO_PIN_MASK;
        break;
    case A_GPIO_OUT_W1TC:
        s->out &= ~v;
        break;
    case A_GPIO_SDIO_SELECT:
        s->sdio_select = v;
        return;
    case A_GPIO_ENABLE:
        s->enable = v & ESP32C3_GPIO_PIN_MASK;
        break;
    case A_GPIO_ENABLE_W1TS:
        s->enable |= v & ESP32C3_GPIO_PIN_MASK;
        break;
    case A_GPIO_ENABLE_W1TC:
        s->enable &= ~v;
        break;
    case A_GPIO_STATUS:
        s->status = v & ESP32C3_GPIO_PIN_MASK;
        break;
    case A_GPIO_STATUS_W1TS:
        s->status |= v & ESP32C3_GPIO_PIN_MASK;
        break;
    case A_GPIO_STATUS_W1TC:
        s->status &= ~v;
        break;
    case A_GPIO_CLOCK_GATE:
        s->clock_gate = v & 0x1;
        return;
    case A_GPIO_STRAP:
    case A_GPIO_IN:
    case A_GPIO_PCPU_INT:
    case A_GPIO_PCPU_NMI_INT:
    case A_GPIO_CPUSDIO_INT:
    case A_GPIO_STATUS_NEXT:
    case A_GPIO_DATE:
        /* Read-only on the hardware. */
        return;
    default:
        if (esp32c3_gpio_pin_index(addr, A_GPIO_PIN0, ESP32C3_GPIO_PIN_COUNT,
                                   &index)) {
            s->pin[index] = v;
            break;
        }
        if (esp32c3_gpio_pin_index(addr, A_GPIO_FUNC0_IN_SEL_CFG,
                                   ESP32C3_GPIO_FUNC_IN_COUNT, &index)) {
            s->func_in_sel[index] = v;
            return;
        }
        if (esp32c3_gpio_pin_index(addr, A_GPIO_FUNC0_OUT_SEL_CFG,
                                   ESP32C3_GPIO_PIN_COUNT, &index)) {
            s->func_out_sel[index] = v;
            return;
        }
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented register 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return;
    }

    esp32c3_gpio_update(s);
}

static const MemoryRegionOps esp32c3_gpio_ops = {
    .read = esp32c3_gpio_read,
    .write = esp32c3_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t esp32c3_iomux_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3GPIOState *s = ESP32C3_GPIO(opaque);
    unsigned index;

    if (esp32c3_gpio_pin_index(addr, ESP32C3_IOMUX_PIN0_OFFSET,
                               ESP32C3_GPIO_PIN_COUNT, &index)) {
        return s->iomux[index];
    }
    return 0;
}

static void esp32c3_iomux_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    ESP32C3GPIOState *s = ESP32C3_GPIO(opaque);
    unsigned index;

    if (esp32c3_gpio_pin_index(addr, ESP32C3_IOMUX_PIN0_OFFSET,
                               ESP32C3_GPIO_PIN_COUNT, &index)) {
        s->iomux[index] = (uint32_t)value;
        /* The pull configuration decides what a floating pad reads. */
        esp32c3_gpio_update(s);
    }
}

static const MemoryRegionOps esp32c3_iomux_ops = {
    .read = esp32c3_iomux_read,
    .write = esp32c3_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* An external device driving a pad.  Levels arrive here rather than into
 * GPIO_IN so that an output enable still wins, as it does on the pin. */
static void esp32c3_gpio_set_input(void *opaque, int line, int level)
{
    ESP32C3GPIOState *s = ESP32C3_GPIO(opaque);

    if (line < 0 || line >= ESP32C3_GPIO_PIN_COUNT) {
        return;
    }
    s->ext_valid |= 1u << line;
    if (level) {
        s->ext_level |= 1u << line;
    } else {
        s->ext_level &= ~(1u << line);
    }
    esp32c3_gpio_update(s);
}

static void esp32c3_gpio_reset_hold(Object *obj, ResetType type)
{
    ESP32C3GPIOState *s = ESP32C3_GPIO(obj);

    s->out = 0;
    s->enable = 0;
    s->status = 0;
    s->in = 0;
    s->bt_select = 0;
    s->sdio_select = 0;
    s->clock_gate = 1;
    memset(s->pin, 0, sizeof(s->pin));
    memset(s->iomux, 0, sizeof(s->iomux));
    memset(s->func_in_sel, 0, sizeof(s->func_in_sel));
    memset(s->func_out_sel, 0, sizeof(s->func_out_sel));
    s->ext_level = 0;
    s->ext_valid = 0;

    esp32c3_gpio_update(s);
}

static void esp32c3_gpio_init(Object *obj)
{
    ESP32C3GPIOState *s = ESP32C3_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the property */
    object_property_set_int(obj, "strap_mode", ESP32C3_STRAP_MODE_FLASH_BOOT, &error_fatal);

    /* The C3 register map itself is installed by class_init; the parent has
     * already bound it as MMIO 0.  IO_MUX is MMIO 1. */
    memory_region_init_io(&s->iomux_mem, obj, &esp32c3_iomux_ops, s,
                          TYPE_ESP32C3_GPIO ".iomux", 0x1000);
    sysbus_init_mmio(sbd, &s->iomux_mem);

    sysbus_init_irq(sbd, &s->nmi_irq);
    qdev_init_gpio_in(DEVICE(obj), esp32c3_gpio_set_input,
                      ESP32C3_GPIO_PIN_COUNT);
    qdev_init_gpio_out(DEVICE(obj), s->output, ESP32C3_GPIO_PIN_COUNT);
}

static const VMStateDescription vmstate_esp32c3_gpio = {
    .name = TYPE_ESP32C3_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(out, ESP32C3GPIOState),
        VMSTATE_UINT32(enable, ESP32C3GPIOState),
        VMSTATE_UINT32(status, ESP32C3GPIOState),
        VMSTATE_UINT32(in, ESP32C3GPIOState),
        VMSTATE_UINT32(bt_select, ESP32C3GPIOState),
        VMSTATE_UINT32(sdio_select, ESP32C3GPIOState),
        VMSTATE_UINT32(clock_gate, ESP32C3GPIOState),
        VMSTATE_UINT32_ARRAY(pin, ESP32C3GPIOState, ESP32C3_GPIO_PIN_COUNT),
        VMSTATE_UINT32_ARRAY(iomux, ESP32C3GPIOState, ESP32C3_GPIO_PIN_COUNT),
        VMSTATE_UINT32_ARRAY(func_in_sel, ESP32C3GPIOState,
                             ESP32C3_GPIO_FUNC_IN_COUNT),
        VMSTATE_UINT32_ARRAY(func_out_sel, ESP32C3GPIOState,
                             ESP32C3_GPIO_PIN_COUNT),
        VMSTATE_UINT32(ext_level, ESP32C3GPIOState),
        VMSTATE_UINT32(ext_valid, ESP32C3GPIOState),
        VMSTATE_END_OF_LIST()
    }
};

/* If we need to override any function from the parent (reset, realize, ...), it shall be done
 * in this class_init function */
static void esp32c3_gpio_class_init(ObjectClass *klass, void *data)
{
    static Property esp32c3_gpio_properties[] = {
        /*
         * Which pads share a net.  A test for open drain needs two drivers on
         * one line, and nothing inside the chip puts them there.
         */
        DEFINE_PROP_UINT32("wire", ESP32C3GPIOState, wire, 0),
        DEFINE_PROP_END_OF_LIST(),
    };

    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    Esp32GpioClass *gc = ESP32_GPIO_CLASS(klass);

    gc->ops = &esp32c3_gpio_ops;
    rc->phases.hold = esp32c3_gpio_reset_hold;
    dc->vmsd = &vmstate_esp32c3_gpio;
    device_class_set_props(dc, esp32c3_gpio_properties);
}

static const TypeInfo esp32c3_gpio_info = {
    .name = TYPE_ESP32C3_GPIO,
    .parent = TYPE_ESP32_GPIO,
    .instance_size = sizeof(ESP32C3GPIOState),
    .instance_init = esp32c3_gpio_init,
    .class_init = esp32c3_gpio_class_init,
    .class_size = sizeof(ESP32C3GPIOClass),
};

static void esp32c3_gpio_register_types(void)
{
    type_register_static(&esp32c3_gpio_info);
}

type_init(esp32c3_gpio_register_types)
