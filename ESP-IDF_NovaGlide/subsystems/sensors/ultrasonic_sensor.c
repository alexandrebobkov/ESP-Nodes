#include "ultrasonic_sensor.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ULTRASONIC";

/*static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 1 second
    if ((now - last_read) >= pdMS_TO_TICKS(1000)) {
        float distance;
        esp_err_t res = ultrasonic_measure(&self->sensor, 400.0f, &distance);  // max_distance in cm

        if (res == ESP_OK) {
            self->distance_cm = distance;
            ESP_LOGI(TAG, "Distance: %.2f cm", self->distance_cm);
        } else {
            ESP_LOGW(TAG, "Measurement failed: %d", res);
        }

        last_read = now;
    }
}*/

static esp_err_t ultrasonic_i2c_read(ultrasonic_system_t *self, float *distance_cm) { uint8_t cmd = 0x01; // start measurement uint8_t data[2]; // Send measurement command esp_err_t err = i2c_master_transmit(self->dev, &cmd, 1, pdMS_TO_TICKS(20)); if (err != ESP_OK) { return err; } // Wait for measurement to complete vTaskDelay(pdMS_TO_TICKS(80)); // Read 2 bytes (distance in mm) err = i2c_master_receive(self->dev, data, 2, pdMS_TO_TICKS(20)); if (err != ESP_OK) { return err; } uint16_t raw_mm = (data[0] << 8) | data[1]; *distance_cm = raw_mm / 10.0f; // mm → cm return ESP_OK; }

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) { static TickType_t last_read = 0; if ((now - last_read) >= pdMS_TO_TICKS(1000)) { float distance = 0; esp_err_t res = ultrasonic_i2c_read(self, &distance); if (res == ESP_OK) { self->distance_cm = distance; ESP_LOGI(TAG, "Distance: %.2f cm", distance); } else { ESP_LOGW(TAG, "I2C read failed: %s", esp_err_to_name(res)); } last_read = now; } }


/*void ultrasonic_system_init(ultrasonic_system_t *sys) {
    sys->distance_cm = 0.0f;
    sys->sensor.trigger_pin = ULTRASONIC_TRIGGER_GPIO;
    sys->sensor.echo_pin = ULTRASONIC_ECHO_GPIO;
    sys->update = ultrasonic_update_impl;

    ESP_ERROR_CHECK(ultrasonic_init(&sys->sensor));

    ESP_LOGI(TAG, "Ultrasonic sensor initialized");
    }*/

void ultrasonic_system_init(ultrasonic_system_t *sys, i2c_master_bus_handle_t bus_handle) {
    sys->distance_cm = 0.0f;
    sys->update = ultrasonic_update_impl;
    i2c_device_config_t dev_cfg = {
        .device_address = ULTRASONIC_I2C_ADDR,
        .scl_speed_hz = ULTRASONIC_I2C_SPEED_HZ, };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &sys->dev));
    ESP_LOGI(TAG, "HC-SR04 (I2C mode) registered at 0x%02X", ULTRASONIC_I2C_ADDR);
}
