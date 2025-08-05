#ifndef I2C_IO_H
#define I2C_IO_H

#include <stdio.h>
#include "driver/i2c_master.h"

typedef struct {
    SemaphoreHandle_t i2cMutex;
    uint16_t sda;
    uint16_t scl;
} io_bus;

i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,               // Use I2C_NUM_1 for second port
    .scl_io_num = GPIO_NUM_22,           // Your SCL pin
    .sda_io_num = GPIO_NUM_21,           // Your SDA pin
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true
};

#endif
