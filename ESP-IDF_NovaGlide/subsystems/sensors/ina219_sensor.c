#include "ina219_sensor.h"
#include "i2c_bus.h"  // Use new I2C bus manager
#include "esp_log.h"
#include <string.h>

static const char *TAG = "INA219";

// INA219 registers
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

static i2c_master_dev_handle_t ina219_handle = NULL;

// Read 16-bit register
static esp_err_t ina219_read_reg(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    esp_err_t ret = i2c_bus_read(ina219_handle, reg, data, 2);
    if (ret == ESP_OK) {
        *value = (data[0] << 8) | data[1];
    }
    return ret;
}

// Write 16-bit register
static esp_err_t ina219_write_reg(uint8_t reg, uint16_t value) {
    uint8_t data[2] = {(value >> 8) & 0xFF, value & 0xFF};
    return i2c_bus_write(ina219_handle, reg, data, 2);
}

static void ina219_update_impl(ina219_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    if ((now - last_read) >= pdMS_TO_TICKS(2500)) {
        uint16_t shunt_raw, bus_raw, power_raw, current_raw;

        if (ina219_read_reg(INA219_REG_BUSVOLTAGE, &bus_raw) == ESP_OK) {
            self->bus_voltage = (bus_raw >> 3) * 0.004f;  // 4mV per bit
        }

        if (ina219_read_reg(INA219_REG_SHUNTVOLTAGE, &shunt_raw) == ESP_OK) {
            self->shunt_voltage = (int16_t)shunt_raw * 0.00001f;  // 10uV per bit
        }

        if (ina219_read_reg(INA219_REG_CURRENT, &current_raw) == ESP_OK) {
            self->current = (int16_t)current_raw * 0.001f;  // 1mA per bit
        }

        if (ina219_read_reg(INA219_REG_POWER, &power_raw) == ESP_OK) {
            self->power = power_raw * 0.02f;  // 20mW per bit
        }

        ESP_LOGI(TAG, "VBUS: %.2fV, I: %.2fmA, P: %.2fmW",
                 self->bus_voltage, self->current * 1000, self->power * 1000);

        last_read = now;
    }
}

void ina219_system_init(ina219_system_t *sys) {
    sys->bus_voltage = 0.0f;
    sys->shunt_voltage = 0.0f;
    sys->current = 0.0f;
    sys->power = 0.0f;
    sys->update = ina219_update_impl;

    // Add INA219 to I2C bus
    esp_err_t ret = i2c_bus_add_device(I2C_ADDR, "INA219", &ina219_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add INA219 to I2C bus");
        return;
    }

    // Configure INA219
    uint16_t config = 0x399F;  // 16V range, 320mV shunt range, 12-bit, continuous
    ina219_write_reg(INA219_REG_CONFIG, config);

    // Calibration for 100mOhm shunt: Cal = 0.04096 / (Current_LSB * Rshunt)
    uint16_t cal = 4096;
    ina219_write_reg(INA219_REG_CALIBRATION, cal);

    ESP_LOGI(TAG, "INA219 initialized");
}
