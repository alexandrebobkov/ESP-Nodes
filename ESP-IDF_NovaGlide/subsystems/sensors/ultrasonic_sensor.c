#include "ultrasonic_sensor.h"
#include "esp_log.h"

static const char *TAG = "ULTRASONIC";

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
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
}

void ultrasonic_system_init(ultrasonic_system_t *sys) {
    sys->distance_cm = 0.0f;
    sys->sensor.trigger_pin = ULTRASONIC_TRIGGER_GPIO;
    sys->sensor.echo_pin = ULTRASONIC_ECHO_GPIO;
    sys->update = ultrasonic_update_impl;

    ESP_ERROR_CHECK(ultrasonic_init(&sys->sensor));

    ESP_LOGI(TAG, "Ultrasonic sensor initialized");
}
