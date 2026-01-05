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

void ultrasonic_system_init(ultrasonic_system_t *sys)
{
        sys->distance_cm = 0.0f;
        sys->update = ultrasonic_update_impl;

        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_SDA_PIN,
            .scl_io_num = I2C_SCL_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_FREQ_HZ,
        };

        ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));

        ESP_LOGI(TAG, "HC-SR04 (I2C mode) initialized");
}
