#include "ultrasonic_sensor.h"
#include "esp_log.h"

static const char *TAG = "ULTRASONIC";

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 1 second
    if ((now - last_read) >= pdMS_TO_TICKS(1000)) {
        uint32_t time_us;
        esp_err_t res = ultrasonic_measure(&self->sensor, 100, &time_us);  // Changed from ultrasonic_measure_raw

        if (res == ESP_OK) {
            self->distance_cm = (float)time_us / ROUNDTRIP_CM;
            ESP_LOGI(TAG, "Distance: %.2f cm", self->distance_cm);
        }

        last_read = now;
    }
}

void ultrasonic_system_init(ultrasonic_system_t *sys) {
    sys->distance_cm = 0.0f;
    sys->sensor.trigger_pin = ULTRASONIC_TRIGGER_GPIO;  // Changed from trigger_gpio
    sys->sensor.echo_pin = ULTRASONIC_ECHO_GPIO;        // Changed from echo_gpio
    sys->update = ultrasonic_update_impl;

    ESP_ERROR_CHECK(ultrasonic_init(&sys->sensor));

    ESP_LOGI(TAG, "Ultrasonic sensor initialized");
}
