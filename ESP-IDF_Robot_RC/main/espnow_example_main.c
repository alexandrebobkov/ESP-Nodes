/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
//#include "esp_netif.h"
//#include "esp_wifi.h"
#include "esp_log.h"
//#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
//#include "espnow_example.h"

#include "rc.h"
#include "motor_controls.h"
#include "controls.h"

#include "controller.h"
#include "receiver.h"
#include "common.h"
#include "config.h"


#define ESPNOW_MAXDELAY 512

static esp_now_peer_info_t peerInfo;
static sensors_data_t *buf;
static sensors_data_t *buffer;
static const char *TAG = "Remote Controller";

void app_main(void)
{
    /*
        ADC
    */
   //rc_adc_init();
   //xTaskCreate(rc_task, "RC", 2048, NULL, 5, NULL);

   // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    //esp_now_init();
    rc_espnow_init();
    esp_now_register_recv_cb(onDataReceived);
    esp_now_register_send_cb(onDataSent);

    memcpy (peerInfo.peer_addr, receiver_mac, 6);
    esp_now_add_peer(&peerInfo);

    xTaskCreate (rc_send_data_task2, "RC", 2048, NULL, 15, NULL);
}
