#include "ultrasonic.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ULTRASONIC";

// Common I2C ultrasonic sensor register addresses
// These vary by sensor model - adjust based on your datasheet
#define ULTRASONIC_REG_DISTANCE_H  0x00  // High byte of distance
#define ULTRASONIC_REG_DISTANCE_L  0x01  // Low byte of distance
#define ULTRASONIC_REG_TRIGGER     0x02  // Trigger measurement register

// Alternative: Some sensors use a single command byte
#define ULTRASONIC_CMD_MEASURE     0x51  // Trigger measurement command

static i2c_master_dev_handle_t ultrasonic_handle = NULL;

// Read distance from sensor (16-bit value)
static esp_err_t ultrasonic_read_distance(uint16_t *distance) {
    uint8_t data[2];

    // Method 1: Read from distance registers
    esp_err_t ret = i2c_bus_read(ultrasonic_handle, ULTRASONIC_REG_DISTANCE_H, data, 2);
    if (ret == ESP_OK) {
        *distance = (data[0] << 8) | data[1];
    }

    return ret;
}

// Trigger a measurement
static esp_err_t ultrasonic_trigger_measurement(void) {
    // Method 1: Write to trigger register
    uint8_t trigger = 0x01;
    esp_err_t ret = i2c_bus_write(ultrasonic_handle, ULTRASONIC_REG_TRIGGER, &trigger, 1);

    // Method 2 (alternative): Send measurement command directly
    // Uncomment if your sensor uses command-based triggering:
    // uint8_t cmd = ULTRASONIC_CMD_MEASURE;
    // esp_err_t ret = i2c_master_transmit(ultrasonic_handle, &cmd, 1, 1000 / portTICK_PERIOD_MS);

    return ret;
}

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 100ms (adjust as needed)
    if ((now - last_read) < pdMS_TO_TICKS(100)) {
        return;
    }
    last_read = now;

    // Trigger a measurement
    esp_err_t ret = ultrasonic_trigger_measurement();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to trigger measurement: %s", esp_err_to_name(ret));
        self->measurement_valid = false;
        return;
    }

    // Wait for measurement to complete (typical: 30-100ms)
    vTaskDelay(pdMS_TO_TICKS(50));

    // Read the distance
    uint16_t raw_distance;
    ret = ultrasonic_read_distance(&raw_distance);

    if (ret == ESP_OK) {
        // Convert raw value to cm
        // Conversion depends on sensor model:
        // - Some sensors return mm (divide by 10)
        // - Some return cm directly
        // - Some use custom scaling
        // Adjust this formula based on your sensor's datasheet:
        self->distance_cm = (float)raw_distance / 10.0f;  // Assuming raw is in mm

        // Sanity check (typical range: 2-400cm)
        if (self->distance_cm >= 2.0f && self->distance_cm <= 400.0f) {
            self->measurement_valid = true;
            ESP_LOGI(TAG, "Distance: %.2f cm (raw: %u)", self->distance_cm, raw_distance);
        } else {
            self->measurement_valid = false;
            ESP_LOGW(TAG, "Distance out of range: %.2f cm", self->distance_cm);
        }
    } else {
        self->measurement_valid = false;
        ESP_LOGW(TAG, "Failed to read distance: %s", esp_err_to_name(ret));
    }
}

void ultrasonic_system_init(ultrasonic_system_t *sys) {
    sys->distance_cm = 0.0f;
    sys->measurement_valid = false;
    sys->update = ultrasonic_update_impl;

    // Add ultrasonic sensor to I2C bus
    esp_err_t ret = i2c_bus_add_device(ULTRASONIC_I2C_ADDR, "ULTRASONIC", &ultrasonic_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ultrasonic sensor to I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "I2C ultrasonic sensor initialized at address 0x%02X", ULTRASONIC_I2C_ADDR);

    // Some sensors may need initialization commands - add here if needed
    // Example:
    // uint8_t init_cmd[] = {0x00, 0x01};
    // i2c_bus_write(ultrasonic_handle, 0x00, init_cmd, sizeof(init_cmd));
}
