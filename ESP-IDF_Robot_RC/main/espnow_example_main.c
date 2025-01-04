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
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"

#include "rc.h"
#include "motor_controls.h"
#include "controls.h"

#include "controller.h"
#include "receiver.h"
#include "common.h"
#include "config.h"

/*#define PROJ_X                      (1)                     // ADC1_CH1; 0 GPIO joystick, x-axis
#define PROJ_Y                      (0)                     // ADC1_CH0; 1 GPIO joystick, y-axis
#define NAV_BTN                     (8)                     // 8 GPIO joystick button*/




//static uint8_t receiver_mac[ESP_NOW_ETH_ALEN]   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static esp_now_peer_info_t peerInfo;
static uint8_t flagToSend = 0;


static sensors_data_t *buf;
static sensors_data_t *buffer;

static void rc_send_data_task2 (void *pvParameter);

#define ESPNOW_MAXDELAY 512

static const char *TAG = "Remote Controller";

static QueueHandle_t s_example_espnow_queue;

// Broadcast address
//static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45 };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };



/* WiFi should start before using ESPNOW */
/*static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}*/

static void rc_task (void *arg) {
    while (true) {
        rc_get_raw_data();

        ESP_LOGI("PWM", "Motor 1 PWM: %d", m.motor1_rpm_pcm);
        ESP_LOGI("PWM", "Motor 2 PWM: %d", m.motor2_rpm_pcm);
        ESP_LOGI("PWM", "Motor 3 PWM: %d", m.motor3_rpm_pcm);
        ESP_LOGI("PWM", "Motor 4 PWM: %d", m.motor4_rpm_pcm);

        //vTaskDelay (10 / portTICK_PERIOD_MS);  // Determines responsiveness  
        vTaskDelay (1000 / portTICK_PERIOD_MS); 
    }
}

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
    esp_now_init();
    esp_now_register_recv_cb(onDataReceived);
    esp_now_register_send_cb(onDataSent);

    memcpy (peerInfo.peer_addr, receiver_mac, 6);
    esp_now_add_peer(&peerInfo);
    if (esp_now_is_peer_exist(receiver_mac)) {
        ESP_LOGI("ESP-NOW", "Receiver exists.");
        sendData();
    }
    else
        ESP_LOGE("ESP-NOW", "Receiver does not exists.");
    xTaskCreate (rc_send_data_task, "RC", 2048, NULL, 15, NULL);
}
