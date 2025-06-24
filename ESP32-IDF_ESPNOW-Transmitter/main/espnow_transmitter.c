/*  ESPNOW Transmitter
    by: Alexander Bobkov
    Date Created:   June 17, 2025
    Updated:        June 18, 2025
    SDK:abort       ESP-IDF v.5.4.1

    Modification:   added boolean variable LED to the struct 'buffer'

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This program uses ESPNOW for transmitting joystick x- and y- axis values to the receiving device.
*/
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "esp_err.h"


#include "joystick.h"

#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "esp_err.h"

static temperature_sensor_handle_t temp_sensor;
const char *TAGt = "ESP-NOW_Transmitter"; 

/*
    EXP32-C3 Chip built-in temprature sensor
    Read & display the temperature value
*/
static void temp_sensor_task (void *arg) {
    while (true) {
        ESP_LOGI(TAGt, "Reading sensor temperature");
        float tsens_value;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
        ESP_LOGW(TAGt, "Temperature value %.02f ℃", tsens_value);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
static void chip_sensor_init () {
    temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    ESP_LOGI(TAGt, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
}

void app_main(void)
{
    // Initialize internal temperature sensor
    chip_sensor_init();
    xTaskCreate(temp_sensor_task, "ESP32C3 Sensor", 2048, NULL, 15, NULL);

    joystick_adc_init();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    wifi_init();
    transmission_init();
    // Defince a task for periodically sending ESPNOW remote control data
    xTaskCreate(rc_send_data_task, "RC", 2048, NULL, 4, NULL);
}