#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>

#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_SCL_IO 2
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 100

#define I2C_MAX_DEVICES 8

typedef struct {
    uint8_t address;
    i2c_master_dev_handle_t dev_handle;
    bool is_active;
    const char *name;
} i2c_device_t;

esp_err_t i2c_bus_init(void);
i2c_master_bus_handle_t i2c_bus_get(void);

esp_err_t i2c_bus_add_device(uint8_t address, const char *name,
                             i2c_master_dev_handle_t *dev_handle);

esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                        const uint8_t *data, size_t len);

esp_err_t i2c_bus_write_byte(i2c_master_dev_handle_t dev_handle,
                             uint8_t reg_addr, uint8_t value);

esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                       uint8_t *data, size_t len);

esp_err_t i2c_bus_read_byte(i2c_master_dev_handle_t dev_handle,
                            uint8_t reg_addr, uint8_t *value);

bool i2c_bus_device_present(uint8_t address);
int i2c_bus_scan(void);

esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle);
esp_err_t i2c_bus_deinit(void);
