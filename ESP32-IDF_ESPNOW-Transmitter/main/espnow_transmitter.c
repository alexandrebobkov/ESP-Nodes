/*  ESPNOW Transmitter
    by: Alexander Bobkov
    Date Created:   June 17, 2025
    Updated:        June 18, 2025
                    July 29, 2025 (added automatic channel selection when ESPNOW transmission fails)
    SDK:            ESP-IDF v.5.4.1

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

#include "system_health.h"
#include "joystick.h"

void app_main(void)
{
    // Initialize internal temperature sensor
    chip_sensor_init();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    wifi_init();
    joystick_adc_init();
    transmission_init();

    system_led_init();
}