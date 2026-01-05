// ============================================================================
// i2c_bus.h
// ============================================================================
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
    uint8_t address;                        // 7-bit I2C address
    i2c_master_dev_handle_t dev_handle;     // ESP-IDF device handle
    bool is_active;                         // Device registration status
    const char *name;                       // Device name for logging
} i2c_device_t;

/**
 * @brief Initialize the I2C bus
 *
 * This must be called once before using any I2C devices.
 * It initializes the I2C master bus with the configured pins and frequency.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Add an I2C device to the bus
 *
 * Registers a new I2C device and returns a handle for future operations.
 * Multiple devices can share the same bus.
 *
 * @param address 7-bit I2C device address
 * @param name Device name for logging (e.g., "INA219", "BMP280")
 * @param dev_handle Pointer to store the device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_add_device(uint8_t address, const char *name, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Write data to an I2C device
 *
 * @param dev_handle Device handle obtained from i2c_bus_add_device()
 * @param reg_addr Register address to write to
 * @param data Pointer to data buffer
 * @param len Length of data to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                        const uint8_t *data, size_t len);

/**
 * @brief Write a single byte to an I2C device register
 *
 * @param dev_handle Device handle
 * @param reg_addr Register address
 * @param value Value to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t value);

/**
 * @brief Read data from an I2C device
 *
 * @param dev_handle Device handle obtained from i2c_bus_add_device()
 * @param reg_addr Register address to read from
 * @param data Pointer to buffer to store read data
 * @param len Length of data to read
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                       uint8_t *data, size_t len);

/**
 * @brief Read a single byte from an I2C device register
 *
 * @param dev_handle Device handle
 * @param reg_addr Register address
 * @param value Pointer to store the read value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_read_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *value);

/**
 * @brief Scan the I2C bus for devices
 *
 * Scans all possible 7-bit addresses (0x08 to 0x77) and logs found devices.
 * Useful for debugging and device discovery.
 *
 * @return Number of devices found
 */
int i2c_bus_scan(void);

/**
 * @brief Check if a device is present on the bus
 *
 * @param address 7-bit I2C address to check
 * @return true if device responds, false otherwise
 */
bool i2c_bus_device_present(uint8_t address);

/**
 * @brief Remove an I2C device from the bus
 *
 * @param dev_handle Device handle to remove
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Deinitialize the I2C bus
 *
 * Cleans up all resources. Call this before restarting the system.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_deinit(void);

#endif // I2C_BUS_H
