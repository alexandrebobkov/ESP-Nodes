#include "ultrasonic_sensor.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ULTRASONIC";

// Command bytes for your sensor (based on test results)
#define ULTRASONIC_CMD_MEASURE_CM    0x50  // Measure in centimeters
#define ULTRASONIC_CMD_MEASURE_INCH  0x51  // Measure in inches
#define ULTRASONIC_CMD_MEASURE_US    0x52  // Measure in microseconds

static i2c_master_dev_handle_t ultrasonic_handle = NULL;

// Trigger measurement and read distance
static esp_err_t ultrasonic_measure_distance(uint16_t *distance) {
    // CRITICAL: This sensor requires initialization commands before EVERY measurement
    // Without this, only the first measurement after power-on works
    uint8_t init_cmds[] = {0x00, 0x01};
    for (int i = 0; i < sizeof(init_cmds); i++) {
        esp_err_t ret = i2c_master_transmit(ultrasonic_handle, &init_cmds[i], 1, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Init cmd 0x%02X failed (may be normal)", init_cmds[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Send measurement command (0x50 = cm)
    uint8_t cmd = ULTRASONIC_CMD_MEASURE_CM;
    esp_err_t ret = i2c_master_transmit(ultrasonic_handle, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    // Wait for measurement to complete (70ms typical)
    vTaskDelay(pdMS_TO_TICKS(70));

    // Read 4 bytes (distance is in first 2 bytes)
    uint8_t data[4];
    ret = i2c_master_receive(ultrasonic_handle, data, 4, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    // Extract 16-bit distance from first 2 bytes
    *distance = (data[0] << 8) | data[1];

    return ESP_OK;
}

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 100ms (adjust as needed)
    if ((now - last_read) < pdMS_TO_TICKS(100)) {
        return;
    }
    last_read = now;

    // Measure distance
    uint16_t raw_distance;
    esp_err_t ret = ultrasonic_measure_distance(&raw_distance);

    if (ret == ESP_OK) {
        // Distance is already in cm (command 0x50)
        self->distance_cm = (float)raw_distance;

        // Check for valid range (0 means no detection, typical range 2-400cm)
        if (raw_distance == 0) {
            self->measurement_valid = false;
            ESP_LOGW(TAG, "No object detected (distance = 0)");
        } else if (self->distance_cm >= 2.0f && self->distance_cm <= 400.0f) {
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
    ESP_LOGI(TAG, "Protocol: Command-based (0x50=cm, 0x51=inch, 0x52=µs)");

    // Test initial read
    vTaskDelay(pdMS_TO_TICKS(100));
    uint16_t test_distance;
    ret = ultrasonic_measure_distance(&test_distance);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial test read: %u cm", test_distance);
    } else {
        ESP_LOGW(TAG, "Initial test read failed (this is normal on startup)");
    }
}
