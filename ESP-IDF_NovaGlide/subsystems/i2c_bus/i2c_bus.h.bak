#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// I2C Bus Configuration
#define I2C_MASTER_SCL_IO           2       // GPIO for SCL
#define I2C_MASTER_SDA_IO           3       // GPIO for SDA
#define I2C_MASTER_FREQ_HZ          100000  // 100kHz standard mode
#define I2C_MASTER_TIMEOUT_MS       1000    // Timeout for I2C operations

// Maximum number of devices that can be registered
#define I2C_MAX_DEVICES             8

// I2C device handle structure
typedef struct {
    uint8_t address;
    i2c_master_dev_handle_t dev_handle;
    bool is_active;
    const char *name;
} i2c_device_t;

/**
 * @brief Initialize the I2C bus
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Add an I2C device to the bus
 */
esp_err_t i2c_bus_add_device(uint8_t address, const char *name, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Write data to an I2C device
 */
esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                        const uint8_t *data, size_t len);

/**
 * @brief Write a single byte to an I2C device register
 */
esp_err_t i2c_bus_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t value);

/**
 * @brief Read data from an I2C device
 */
esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                       uint8_t *data, size_t len);

/**
 * @brief Read a single byte from an I2C device register
 */
esp_err_t i2c_bus_read_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *value);

/**
 * @brief Scan the I2C bus for devices
 */
int i2c_bus_scan(void);

/**
 * @brief Check if a device is present on the bus
 */
bool i2c_bus_device_present(uint8_t address);

/**
 * @brief Remove an I2C device from the bus
 */
esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Deinitialize the I2C bus
 */
esp_err_t i2c_bus_deinit(void);

#endif // I2C_BUS_H
