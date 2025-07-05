/*  ESPNOW Receiver
    by: Alexander Bobkov
    Date Created:   July 4, 2025
    Updated:        July 4, 2025
    SDK:            ESP-IDF v.5.4.1

   

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This program uses ESPNOW for receiving joystick x- and y- axis values from the receiving device.
*/

#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "system_health.h"
#include "receiver.h"

void app_main(void) {

        // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    wifi_init();
    esp_now_init();
    esp_now_register_recv_cb((void*)onDataReceived);

    system_led_init();
}