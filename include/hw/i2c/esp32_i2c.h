#ifndef ESP32_I2C_H
#define ESP32_I2C_H

#include "hw/sysbus.h"
#include "qemu/fifo8.h"
#include "hw/i2c/i2c.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_I2C "esp32.i2c"
#define Esp32_I2C(obj) OBJECT_CHECK(Esp32I2CState, (obj), TYPE_ESP32_I2C)
#define Esp32_I2C_GET_CLASS(obj) \
    OBJECT_GET_CLASS(Esp32I2CClass, obj, TYPE_ESP32_I2C)
#define Esp32_I2C_CLASS(klass) \
    OBJECT_CLASS_CHECK(Esp32I2CClass, klass, TYPE_ESP32_I2C)

/* The C-series parts renumbered the command opcodes; see the opcode maps in
 * esp32_i2c.c.  Everything else about the controller is the same, so this is a
 * subclass rather than a device of its own. */
#define TYPE_ESP32C3_I2C "esp32c3.i2c"


#define ESP32_I2C_MEM_SIZE 0x100
#define ESP32_I2C_FIFO_LENGTH 32
#define ESP32_I2C_CMD_COUNT 16


typedef struct Esp32I2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;
    Fifo8 rx_fifo;
    Fifo8 tx_fifo;
    bool trans_ongoing;

    uint32_t ctr_reg;
    uint32_t timeout_reg;
    uint32_t int_ena_reg;
    uint32_t int_raw_reg;
    uint32_t sda_hold_reg;
    uint32_t sda_sample_reg;
    uint32_t high_period_reg;
    uint32_t low_period_reg;
    uint32_t start_hold_reg;
    uint32_t rstart_setup_reg;
    uint32_t stop_hold_reg;
    uint32_t stop_setup_reg;
    uint32_t cmd_reg[ESP32_I2C_CMD_COUNT];
} Esp32I2CState;

typedef struct Esp32I2CClass {
    SysBusDeviceClass parent_class;

    /* I2C_CMD.OPCODE as the chip numbers it, mapped to the canonical opcode
     * the transaction engine switches on.  Indexed by the raw three-bit
     * field, so it always has eight entries. */
    const uint8_t *opcodes;

    /* How many command registers the chip has.  The original ESP32 has 16;
     * the C3 has 8, and real registers of its own where the other 8 would be,
     * so treating the whole range as commands on that part would both execute
     * a timeout configuration as a command and lose the configuration. */
    unsigned cmd_count;
} Esp32I2CClass;


REG32(I2C_CTR, 0x04);
    FIELD(I2C_CTR, MS_MODE, 4, 1);
    FIELD(I2C_CTR, TRANS_START, 5, 1);

REG32(I2C_STATUS, 0x08);
    FIELD(I2C_STATUS, BUS_BUSY, 4, 1);
    FIELD(I2C_STATUS, RXFIFO_CNT, 8, 6);
    FIELD(I2C_STATUS, TXFIFO_CNT, 18, 6);

REG32(I2C_TIMEOUT, 0x0c);

REG32(I2C_FIFO_CONF, 0x18);
    FIELD(I2C_FIFO_CONF, NONFIFO_EN, 10, 1);
    FIELD(I2C_FIFO_CONF, RX_FIFO_RST, 12, 1);
    FIELD(I2C_FIFO_CONF, TX_FIFO_RST, 13, 1);

REG32(I2C_FIFO_DATA, 0x1c);

REG32(I2C_INT_RAW, 0x20);
    FIELD(I2C_INT_RAW, ACK_ERR, 10, 1);
    FIELD(I2C_INT_RAW, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_RAW, END_DETECT, 3, 1);

REG32(I2C_INT_CLR, 0x24);
    FIELD(I2C_INT_CLR, ACK_ERR, 10, 1);
    FIELD(I2C_INT_CLR, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_CLR, END_DETECT, 3, 1);

REG32(I2C_INT_ENA, 0x28);
    FIELD(I2C_INT_ENA, ACK_ERR, 10, 1);
    FIELD(I2C_INT_ENA, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_ENA, END_DETECT, 3, 1);

REG32(I2C_INT_ST, 0x2c);
    FIELD(I2C_INT_ST, ACK_ERR, 10, 1);
    FIELD(I2C_INT_ST, TRANS_COMPLETE, 7, 1);
    FIELD(I2C_INT_ST, END_DETECT, 3, 1);

REG32(I2C_SDA_HOLD, 0x30);
REG32(I2C_SDA_SAMPLE, 0x34);
REG32(I2C_HIGH_PERIOD, 0x38);
REG32(I2C_LOW_PERIOD, 0x00);  // 0x00 is not a typo
REG32(I2C_START_HOLD, 0x40);
REG32(I2C_RSTART_SETUP, 0x44);
REG32(I2C_STOP_HOLD, 0x48);
REG32(I2C_STOP_SETUP, 0x4c);

REG32(I2C_CMD, 0x58);
    FIELD(I2C_CMD, BYTE_NUM, 0, 8);
    FIELD(I2C_CMD, ACK_CHECK_EN, 8, 1);
    FIELD(I2C_CMD, ACK_EXP, 9, 1);
    FIELD(I2C_CMD, ACK_VAL, 10, 1);
    FIELD(I2C_CMD, OPCODE, 11, 3);
    FIELD(I2C_CMD, DONE, 31, 1);
/* 15 more command registers omitted */

/* Canonical opcodes.  These are what the transaction engine works in; what a
 * given chip puts in I2C_CMD.OPCODE is mapped to one of these by the class's
 * opcode table.  The values happen to match the original ESP32's numbering,
 * which is where they came from. */
typedef enum {
    I2C_OPCODE_RSTART  = 0,
    I2C_OPCODE_WRITE   = 1,
    I2C_OPCODE_READ    = 2,
    I2C_OPCODE_STOP    = 3,
    I2C_OPCODE_END     = 4,
    I2C_OPCODE_INVALID = 0xff,
} i2c_opcode_t;

#endif /* ESP32_I2C_H */
