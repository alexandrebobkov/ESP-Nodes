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


#define ESPNOW_MAXDELAY 512

typedef struct {
    uint8_t     type;                       // Broadcast or unicast ESPNOW data.s
    uint16_t    seq_num;                     // Sequence number of ESPNOW data.
    uint16_t    crc;                         // CRC16 value of ESPNOW data.
    uint8_t     x_axis;
    uint8_t     y_axis;
    bool        nav_bttn;
    uint8_t     motor1_rpm_pcm;
    uint8_t     motor2_rpm_pcm;
    uint8_t     motor3_rpm_pcm;
    uint8_t     motor4_rpm_pcm;
} __attribute__((packed)) sensors_data_t;

typedef struct {
    int len;                                // Length of ESPNOW data to be sent, unit: byte.
    uint8_t     *buffer;                      // Buffer; pointer to the data struct.
    uint8_t     dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} espnow_data_packet_t;

static uint8_t receiver_mac[ESP_NOW_ETH_ALEN]   = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};

static esp_now_peer_info_t peerInfo;
static sensors_data_t *buf, *buffer;
static const char *TAG = "Remote Controller";

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

void onDataReceived (uint8_t *mac_addr, uint8_t *data, uint8_t data_len) {

    //memcpy(buf, data, data_len);
    buf = (sensors_data_t*)data;
    ESP_LOGW(TAG, "Data was received");
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->x_axis);
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->y_axis);
    ESP_LOGI(TAG, "PCM 1: 0x%04x", buf->motor1_rpm_pcm);
}
void onDataSent (uint8_t *mac_addr, esp_now_send_status_t status) {
    //status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    ESP_LOGW(TAG, "Packet send status: %i", status);
}
void sensors_data_prepare(espnow_data_packet_t *send_packet) {
    //sensors_data_t *buffer;
    //malloc(sizeof(sensors_data_t));
    //send_packet->buffer = &buffer;
    //sensors_data_t *buffer = (sensors_data_t *)send_packet->buffer;
    sensors_data_t *buffer = (sensors_data_t *)send_packet->buffer;
    assert(send_packet->len >= sizeof(sensors_data_t));

    buffer->type = 1;
    buffer->crc = 0;
    buffer->x_axis = 0;
    buffer->y_axis = 0;
    buffer->nav_bttn = 0;
    buffer->motor1_rpm_pcm = 0;
    buffer->motor2_rpm_pcm = 0;
    buffer->motor3_rpm_pcm = 0;
    buffer->motor4_rpm_pcm = 0;
    ESP_LOGW(TAG, "x-axis: %x", (uint8_t)buffer->x_axis);
    buffer->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buffer, send_packet->len);
}
static void rc_send_data_task2 (void *pvParameter) {

    espnow_data_packet_t *send_packet = (espnow_data_packet_t *)pvParameter;

    while (true) {
        //memcpy(send_packet->dest_mac, receiver_mac, ESP_NOW_ETH_ALEN);
        esp_err_t r = esp_now_send(receiver_mac, send_packet->buffer, sizeof(sensors_data_t));//send_packet->len);
        //esp_now_send(send_packet->dest_mac, send_packet->buffer, send_packet->len);

        if (r != ESP_OK) {
            ESP_LOGE(TAG, "Send error.");
            vTaskDelete(NULL);
            break;
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
static void rc_send_data_task (void *arg) {

    while (true) {
        if (esp_now_is_peer_exist(receiver_mac)) {
            sendData();
        }
        vTaskDelay (1000 / portTICK_PERIOD_MS);
    }
}
static esp_err_t rc_espnow_init (void) {

    espnow_data_packet_t *send_packet;

    send_packet = malloc(sizeof(espnow_data_packet_t));
    if (send_packet == NULL) {
        ESP_LOGE(TAG, "malloc fail.");
        return ESP_FAIL;
    }

    memset(send_packet, 0, sizeof(espnow_data_packet_t));
    memcpy(send_packet->dest_mac, receiver_mac, ESP_NOW_ETH_ALEN);
    send_packet->len = CONFIG_ESPNOW_SEND_LEN; // 128
    send_packet->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    sensors_data_prepare(send_packet);
    xTaskCreate(rc_send_data_task2, "controller data packets task", 2048, send_packet, 8, NULL);

    return ESP_OK;
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
    rc_espnow_init();
    esp_now_register_recv_cb(onDataReceived);
    esp_now_register_send_cb(onDataSent);

    memcpy (peerInfo.peer_addr, receiver_mac, 6);
    esp_now_add_peer(&peerInfo);

    //xTaskCreate (rc_send_data_task, "RC", 2048, NULL, 15, NULL);
}
