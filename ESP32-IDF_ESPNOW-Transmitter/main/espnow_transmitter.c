/*  ESPNOW Transmitter
    by: Alexander Bobkov
    Date Created:   June 17, 2025
    Updated:        June 17, 2025
    SDK:abort       ESP-IDF v.5.4.1

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This program uses ESPNOW for transmitting joystick x- and y- axis values to the receiving device.
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

#include "joystick.h"
#include "config.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "ESP-NOW_Transmitter";
static sensors_data_t buffer; 

/* WiFi should start before using ESPNOW */
static void wifi_init() {
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
}

// Function to delete peer (i.e. when communication error occurs)
void deletePeer (void) {
    uint8_t delStatus = esp_now_del_peer(receiver_mac);
    if (delStatus != 0) {
        ESP_LOGE("ESP-NOW", "Could not delete peer");
    }
}
static void statusDataSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Data sent successfully to: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGE(TAG, "Error sending data to: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        ESP_LOGE("sendData()", "Error sending data. Error code: 0x%04X", status);
        ESP_LOGE("sendData()", "esp_now_send() failed: %s", esp_err_to_name(status));
        ESP_LOGE("sendData()", "Ensure that receiver is powered-on and MAC is correct.");
        deletePeer();
    }
}
// Function for sending the data to the receiver
void sendData (void) {
    buffer.crc = 0;
    buffer.x_axis = 240;
    buffer.y_axis = 256;
    buffer.nav_bttn = 0;
    buffer.motor1_rpm_pcm = 0; //10;
    buffer.motor2_rpm_pcm = 0;
    buffer.motor3_rpm_pcm = 0;
    buffer.motor4_rpm_pcm = 0;

    get_joystick_xy(&x, &y);
    //ESP_LOGI("(x, y)", "[ %d, %d ]", x, y);
    buffer.x_axis = x;
    buffer.y_axis = y;

    // Display brief summary of data being sent.
    ESP_LOGI(TAG, "Joystick (x,y) position ( %d, %d )", buffer.x_axis, buffer.y_axis);  
    ESP_LOGI(TAG, "pcm 1, pcm 2 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor1_rpm_pcm, (uint8_t)buffer.motor2_rpm_pcm);
    ESP_LOGI(TAG, "pcm 3, pcm 4 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor3_rpm_pcm, (uint8_t)buffer.motor4_rpm_pcm);

    // Call ESP-NOW function to send data (MAC address of receiver, pointer to the memory holding data & data length)
    uint8_t result = esp_now_send(receiver_mac, (uint8_t *)&buffer, sizeof(buffer));

    // If status is NOT OK, display error message and error code (in hexadecimal).
    if (result != 0) {
        ESP_LOGE("sendData()", "Error sending data! Error code: 0x%04X", result);
        ESP_LOGE("sendData()", "esp_now_send() failed: %s", esp_err_to_name(result));
        ESP_LOGE("sendData()", "Ensure that receiver is powered-on.");
        ESP_LOGE("sendData()", "Ensure that received MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
                 receiver_mac[0], receiver_mac[1], receiver_mac[2],
                 receiver_mac[3], receiver_mac[4], receiver_mac[5]);
        deletePeer();
    }
}
static void rc_send_data_task()
{
    while (true) {
        if (esp_now_is_peer_exist(receiver_mac))
            sendData();
        vTaskDelay (250 / portTICK_PERIOD_MS);
    }
}

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

    esp_err_t espnow_ret = esp_now_init();
    if (espnow_ret != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing ESPNOW");
        ESP_LOGE(TAG, "esp_now_init() failed: %s", esp_err_to_name(espnow_ret));
        return;
    }
    ESP_LOGI(TAG, "ESPNOW initialized successfully");
    esp_now_register_send_cb(statusDataSend);

    // Set ESP-NOW receiver device configuration values
    memcpy(devices.peer_addr, receiver_mac, 6);
    devices.channel = 1;
    devices.encrypt = false;
    esp_now_add_peer(&devices);

    // Defince a task for periodically sending ESPNOW remote control data
    xTaskCreate(rc_send_data_task, "RC", 2048, NULL, 4, NULL);
}
