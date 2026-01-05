#include "ultrasonic_sensor.h"
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ULTRASONIC";

// I2C address and commands (from working test code)
#define ULTRASONIC_I2C_ADDR         0x57
#define CMD_MEASURE_CM              0x50
#define CMD_MEASURE_INCH            0x51
#define CMD_MEASURE_US              0x52

static i2c_master_dev_handle_t ultrasonic_handle = NULL;

// Measure distance (adapted from working test code)
static esp_err_t ultrasonic_measure_distance(uint16_t *distance) {
    if (ultrasonic_handle == NULL) {
        ESP_LOGE(TAG, "Ultrasonic not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // CRITICAL: Send init commands BEFORE EVERY measurement
    // Without this, only the first measurement works
    uint8_t init_cmds[] = {0x00, 0x01};
    for (int i = 0; i < sizeof(init_cmds); i++) {
        esp_err_t ret = i2c_master_transmit(ultrasonic_handle, &init_cmds[i], 1, 500);
        if (ret != ESP_OK) {
            // Init commands can fail during startup or when sensor is busy - this is normal
            ESP_LOGD(TAG, "Init cmd 0x%02X: %s", init_cmds[i], esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Send measurement command (0x50 = cm)
    uint8_t cmd = CMD_MEASURE_CM;
    esp_err_t ret = i2c_master_transmit(ultrasonic_handle, &cmd, 1, 500);
    if (ret != ESP_OK) {
        // NACK during measurement command usually means sensor can't measure (too close/busy)
        ESP_LOGD(TAG, "Measurement command failed (sensor may be too close or busy)");
        return ESP_ERR_INVALID_RESPONSE;  // Special error code for "too close"
    }

    // Wait for measurement (70ms as in working code)
    vTaskDelay(pdMS_TO_TICKS(70));

    // Read 4 bytes
    uint8_t data[4];
    ret = i2c_master_receive(ultrasonic_handle, data, 4, 500);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Read failed (sensor may be too close or busy)");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Extract distance from first 2 bytes (LITTLE-ENDIAN: low byte first)
    *distance = data[0] | (data[1] << 8);
    //*distance = (data[0] << 8) | (data[1];

    return ESP_OK;
}

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 100ms
    if ((now - last_read) < pdMS_TO_TICKS(100)) {
        return;
    }
    last_read = now;

    // Measure distance
    uint16_t raw_distance;
    esp_err_t ret = ultrasonic_measure_distance(&raw_distance);

    if (ret == ESP_OK) {
        // Convert from mm to cm (sensor returns millimeters)
        self->distance_cm = (float)raw_distance / 10.0f;

        // Check for valid range
        if (raw_distance == 0) {
            self->measurement_valid = false;
            ESP_LOGW(TAG, "No object detected (distance = 0)");
        } else if (self->distance_cm >= 2.0f && self->distance_cm <= 400.0f) {
            self->measurement_valid = true;
            ESP_LOGI(TAG, "Distance: %.2f cm (raw: %u mm)", self->distance_cm, raw_distance);
        } else {
            self->measurement_valid = false;
            ESP_LOGW(TAG, "Distance out of range: %.2f cm", self->distance_cm);
        }
    } else if (ret == ESP_ERR_INVALID_RESPONSE) {
        // Sensor NACKed - object is likely too close (< 2cm) or sensor is busy
        self->measurement_valid = false;
        self->distance_cm = 1.0f;  // Indicate "too close"
        ESP_LOGW(TAG, "Object too close or sensor busy");
    } else {
        self->measurement_valid = false;
        ESP_LOGW(TAG, "Measurement failed");
    }
}

void ultrasonic_system_init(ultrasonic_system_t *sys) {
    sys->distance_cm = 0.0f;
    sys->measurement_valid = false;
    sys->update = ultrasonic_update_impl;

    // Add device to I2C bus using your bus manager
    esp_err_t ret = i2c_bus_add_device(ULTRASONIC_I2C_ADDR, "ULTRASONIC", &ultrasonic_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ultrasonic to I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Ultrasonic sensor initialized at 0x%02X", ULTRASONIC_I2C_ADDR);

    // Send initialization sequence (from working test code)
    ESP_LOGI(TAG, "Sending init commands...");
    uint8_t init_cmds[] = {0x00, 0x01, 0x02, 0xFF};
    for (int i = 0; i < sizeof(init_cmds); i++) {
        i2c_master_transmit(ultrasonic_handle, &init_cmds[i], 1, 500);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Try config register writes (from working test code)
    uint8_t cfg1[] = {0x00, 0x01};
    i2c_master_transmit(ultrasonic_handle, cfg1, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t cfg2[] = {0x01, 0x51};
    i2c_master_transmit(ultrasonic_handle, cfg2, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Test initial measurement
    uint16_t test_distance;
    ret = ultrasonic_measure_distance(&test_distance);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial test: %u cm", test_distance);
    } else {
        ESP_LOGW(TAG, "Initial test failed (normal on startup)");
    }
}
