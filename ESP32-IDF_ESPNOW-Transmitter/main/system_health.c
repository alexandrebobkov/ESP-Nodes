#include "freertos/FreeRTOS.h"
#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "esp_err.h"

#include "system_health.h"

static temperature_sensor_handle_t temp_sensor;
static const char *TAG = "SystemHealth"; 

/*
    EXP32-C3 Chip built-in temprature sensor
    Read & display the temperature value
*/
static void temp_sensor_task (void *arg) {
    while (true) {
        ESP_LOGI(TAG, "Reading sensor temperature");
        float tsens_value;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
        ESP_LOGW(TAG, "Temperature value %.02f ℃", tsens_value);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
void chip_sensor_init () {
    temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    xTaskCreate(temp_sensor_task, "ESP32C3 Sensor", 2048, NULL, 15, NULL);
}