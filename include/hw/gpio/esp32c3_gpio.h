#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"
#include "esp32_gpio.h"

#define TYPE_ESP32C3_GPIO "esp32c3.gpio"
#define ESP32C3_GPIO(obj)           OBJECT_CHECK(ESP32C3GPIOState, (obj), TYPE_ESP32C3_GPIO)
#define ESP32C3_GPIO_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32C3GPIOClass, obj, TYPE_ESP32C3_GPIO)
#define ESP32C3_GPIO_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32C3GPIOClass, klass, TYPE_ESP32C3_GPIO)

/* Bootstrap options for ESP32-C3 (4-bit) */
#define ESP32C3_STRAP_MODE_FLASH_BOOT 0x8   /* SPI Boot */
#define ESP32C3_STRAP_MODE_UART_BOOT  0x2   /* Diagnostic Mode0+UART0 download Mode */
#define ESP32C3_STRAP_MODE_USB_BOOT   0x0   /* Diagnostic Mode1+USB download Mode */

/* The C3 has 22 pads, GPIO0 to GPIO21, so one 32-bit bank rather than the
 * two the original ESP32 needs for its 40. */
#define ESP32C3_GPIO_PIN_COUNT  22
#define ESP32C3_GPIO_PIN_MASK   ((1u << ESP32C3_GPIO_PIN_COUNT) - 1u)

/* Number of peripheral input signals the GPIO matrix can route. */
#define ESP32C3_GPIO_FUNC_IN_COUNT 256

/* Registers, offsets from DR_REG_GPIO_BASE.  A_GPIO_STRAP comes from the
 * parent, which already models it. */
REG32(GPIO_BT_SELECT,   0x0000)
REG32(GPIO_OUT,         0x0004)
REG32(GPIO_OUT_W1TS,    0x0008)
REG32(GPIO_OUT_W1TC,    0x000C)
REG32(GPIO_SDIO_SELECT, 0x001C)
REG32(GPIO_ENABLE,      0x0020)
REG32(GPIO_ENABLE_W1TS, 0x0024)
REG32(GPIO_ENABLE_W1TC, 0x0028)
REG32(GPIO_IN,          0x003C)
REG32(GPIO_STATUS,      0x0044)
REG32(GPIO_STATUS_W1TS, 0x0048)
REG32(GPIO_STATUS_W1TC, 0x004C)
REG32(GPIO_PCPU_INT,    0x005C)
REG32(GPIO_PCPU_NMI_INT, 0x0060)
REG32(GPIO_CPUSDIO_INT, 0x0064)
REG32(GPIO_PIN0,        0x0074)
REG32(GPIO_STATUS_NEXT, 0x014C)
REG32(GPIO_FUNC0_IN_SEL_CFG,  0x0154)
REG32(GPIO_FUNC0_OUT_SEL_CFG, 0x0554)
REG32(GPIO_CLOCK_GATE,  0x062C)
REG32(GPIO_DATE,        0x06FC)

/* Fields of GPIO_PINn.  INT_ENA selects which of the CPU's interrupt inputs
 * the pin's status bit is reported on; bit 0 is the ordinary interrupt and
 * bit 1 the NMI. */
FIELD(GPIO_PIN0, SYNC2_BYPASS,  0, 2)
FIELD(GPIO_PIN0, PAD_DRIVER,    2, 1)
FIELD(GPIO_PIN0, SYNC1_BYPASS,  3, 2)
FIELD(GPIO_PIN0, INT_TYPE,      7, 3)
FIELD(GPIO_PIN0, WAKEUP_ENABLE, 10, 1)
FIELD(GPIO_PIN0, CONFIG,        11, 2)
FIELD(GPIO_PIN0, INT_ENA,       13, 5)

#define ESP32C3_GPIO_INT_ENA_CPU  0x1
#define ESP32C3_GPIO_INT_ENA_NMI  0x2

/* GPIO_PINn INT_TYPE encoding, matching esp-idf's gpio_int_type_t. */
enum {
    ESP32C3_GPIO_INTR_DISABLE   = 0,
    ESP32C3_GPIO_INTR_POSEDGE   = 1,
    ESP32C3_GPIO_INTR_NEGEDGE   = 2,
    ESP32C3_GPIO_INTR_ANYEDGE   = 3,
    ESP32C3_GPIO_INTR_LOW_LEVEL = 4,
    ESP32C3_GPIO_INTR_HIGH_LEVEL = 5,
};

/* IO_MUX, offsets from DR_REG_IO_MUX_BASE.  The pad configuration register
 * for GPIOn is at 0x04 + 4n; 0x00 is the clock gate. */
#define ESP32C3_IOMUX_PIN0_OFFSET  0x0004
REG32(IOMUX_PIN0, ESP32C3_IOMUX_PIN0_OFFSET)
FIELD(IOMUX_PIN0, SLP_OE,  0, 1)
FIELD(IOMUX_PIN0, SLP_SEL, 1, 1)
FIELD(IOMUX_PIN0, SLP_PD,  2, 1)
FIELD(IOMUX_PIN0, SLP_PU,  3, 1)
FIELD(IOMUX_PIN0, SLP_IE,  4, 1)
FIELD(IOMUX_PIN0, SLP_DRV, 5, 2)
FIELD(IOMUX_PIN0, FUN_PD,  7, 1)
FIELD(IOMUX_PIN0, FUN_PU,  8, 1)
FIELD(IOMUX_PIN0, FUN_IE,  9, 1)
FIELD(IOMUX_PIN0, FUN_DRV, 10, 2)
FIELD(IOMUX_PIN0, MCU_SEL, 12, 3)

typedef struct ESP32C3State {
    Esp32GpioState parent;

    /* IO_MUX is a peripheral of its own, but its pull-up and pull-down bits
     * decide what an unconnected pad reads back, so the state that computes
     * GPIO_IN would have to reach across to it on every update.  Modelling
     * both here keeps that in one place. */
    MemoryRegion iomux_mem;

    uint32_t out;
    uint32_t enable;
    uint32_t status;
    uint32_t in;              /* pad level, recomputed rather than written */
    uint32_t bt_select;
    uint32_t sdio_select;
    uint32_t clock_gate;
    uint32_t pin[ESP32C3_GPIO_PIN_COUNT];
    uint32_t iomux[ESP32C3_GPIO_PIN_COUNT];
    uint32_t func_in_sel[ESP32C3_GPIO_FUNC_IN_COUNT];
    uint32_t func_out_sel[ESP32C3_GPIO_PIN_COUNT];

    /* What something outside the SoC is doing to each pad.  ext_valid marks
     * the pins an external device has driven at all; the rest are left to the
     * pull resistors. */
    uint32_t ext_level;
    uint32_t ext_valid;

    qemu_irq nmi_irq;
    qemu_irq output[ESP32C3_GPIO_PIN_COUNT];
} ESP32C3GPIOState;

typedef struct ESP32C3GPIOClass {
    Esp32GpioClass parent;
} ESP32C3GPIOClass;
