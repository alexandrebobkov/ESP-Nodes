#include "ultrasonic_sensor.h"
#include "i2c_bus.h"  // Use our I2C bus manager
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "ULTRASONIC";

// Read distance from I2C ultrasonic sensor
static esp_err_t ultrasonic_i2c_read(ultrasonic_system_t *self, float *distance_cm) {
    uint8_t data[2];

    // Read 2 bytes from register 0x02 (distance in mm)
    esp_err_t err = i2c_bus_read(self->dev, 0x02, data, 2);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t raw_mm = ((uint16_t)data[0] << 8) | data[1];
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
    sys->update = NULL;//ultrasonic_update_impl;

    // Add device to I2C bus
    esp_err_t ret = i2c_bus_add_device(ULTRASONIC_I2C_ADDR, "HC-SR04_I2C", &sys->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ultrasonic sensor to I2C bus");
        return;
    }

    ESP_LOGI(TAG, "HC-SR04 (I2C mode) initialized at 0x%02X", ULTRASONIC_I2C_ADDR);
}

void ultrasonic_probe_registers(ultrasonic_system_t *ultra)
{
    uint8_t buf[2];

    for (uint8_t reg = 0; reg < 8; reg++) {
        esp_err_t err = i2c_bus_read(ultra->dev, reg, buf, 2);
        if (err == ESP_OK) {
            ESP_LOGI("ULTRA_PROBE", "READ reg 0x%02X -> %02X %02X", reg, buf[0], buf[1]);
        } else {
            ESP_LOGI("ULTRA_PROBE", "READ reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        }
    }
}
