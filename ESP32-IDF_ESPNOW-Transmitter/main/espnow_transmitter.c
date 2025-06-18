/*  ESPNOW Transmitter
    by: Alexander Bobkov
    Date Created:   June 17, 2025
    Updated:        June 18, 2025
    SDK:abort       ESP-IDF v.5.4.1

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This program uses ESPNOW for transmitting joystick x- and y- axis values to the receiving device.

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"*/
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

#include "joystick.h"

void app_main(void)
{
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
