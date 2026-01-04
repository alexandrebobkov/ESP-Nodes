#include "temp_sensor.h"
#include "driver/temperature_sensor.h"
#include "esp_log.h"

static const char *TAG = "TEMP_SENSOR";
static temperature_sensor_handle_t s_temp = NULL;

static void temp_update_impl(temp_sensor_system_t *self, TickType_t now)
{
    (void)now;
    float c = 0.0f;
    if (temperature_sensor_get_celsius(s_temp, &c) == ESP_OK) {
        self->last_celsius = c;
    }
}

void temp_sensor_system_init(temp_sensor_system_t *sys)
{
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&cfg, &s_temp));
    ESP_ERROR_CHECK(temperature_sensor_enable(s_temp));

    sys->last_celsius = 0.0f;
    sys->update = temp_update_impl;
}
