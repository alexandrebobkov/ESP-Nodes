#include "temp_sensor.h"
#include "esp_log.h"

static const char *TAG = "TEMP_SENSOR";

static void temp_sensor_update_impl(temp_sensor_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 5 seconds
    if ((now - last_read) >= pdMS_TO_TICKS(5000)) {
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(self->handle, &self->temperature));
        ESP_LOGI(TAG, "Temperature: %.2f°C", self->temperature);
        last_read = now;
    }
}

void temp_sensor_system_init(temp_sensor_system_t *sys) {
    sys->temperature = 0.0f;
    sys->handle = NULL;
    sys->update = temp_sensor_update_impl;

    temperature_sensor_config_t temp_sensor_config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &sys->handle));
    ESP_ERROR_CHECK(temperature_sensor_enable(sys->handle));

    ESP_LOGI(TAG, "Temperature sensor initialized");
}
