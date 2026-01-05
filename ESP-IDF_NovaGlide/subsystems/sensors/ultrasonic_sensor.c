#include "ultrasonic_sensor.h"
#include "i2c_bus.h"  // Use our I2C bus manager
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "ULTRASONIC";

// Read distance from I2C ultrasonic sensor
static esp_err_t ultrasonic_i2c_read(ultrasonic_system_t *self, float *distance_cm) {
    uint8_t cmd = 0x01;  // Start measurement command
    uint8_t data[2];

    // Send measurement command
    esp_err_t err = i2c_master_transmit(self->dev, &cmd, 1, pdMS_TO_TICKS(20));
    if (err != ESP_OK) {
        return err;
    }

    // Wait for measurement to complete (typical: 60-80ms)
    vTaskDelay(pdMS_TO_TICKS(80));

    // Read 2 bytes (distance in mm)
    err = i2c_master_receive(self->dev, data, 2, pdMS_TO_TICKS(20));
    if (err != ESP_OK) {
        return err;
    }

    // Convert to cm
    uint16_t raw_mm = (data[0] << 8) | data[1];
    *distance_cm = raw_mm / 10.0f;  // mm → cm

    return ESP_OK;
}

// Periodic update function
static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 1 second
    if ((now - last_read) >= pdMS_TO_TICKS(1000)) {
        float distance = 0;
        esp_err_t res = ultrasonic_i2c_read(self, &distance);

        if (res == ESP_OK) {
            self->distance_cm = distance;
            ESP_LOGI(TAG, "Distance: %.2f cm", distance);
        } else {
            ESP_LOGW(TAG, "I2C read failed: %s", esp_err_to_name(res));
        }

        last_read = now;
    }
}

// Initialize ultrasonic sensor using I2C bus manager
void ultrasonic_system_init(ultrasonic_system_t *sys) {
    sys->distance_cm = 0.0f;
    sys->update = ultrasonic_update_impl;

    // Add device to I2C bus
    esp_err_t ret = i2c_bus_add_device(ULTRASONIC_I2C_ADDR, "HC-SR04_I2C", &sys->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ultrasonic sensor to I2C bus");
        return;
    }

    ESP_LOGI(TAG, "HC-SR04 (I2C mode) initialized at 0x%02X", ULTRASONIC_I2C_ADDR);
}
