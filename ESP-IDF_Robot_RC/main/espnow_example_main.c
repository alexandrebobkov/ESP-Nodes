/* ESP-NOW Remote Controller & Receiver

   by: Alexander Bobkov
   Jan 4, 2025

   Program that sends values saved in struct from controller device to the receiver using ESP-NOW communication protocol.

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

#include "joystick.h"

// Struct holding sensors values
typedef struct {
    uint16_t    crc;                // CRC16 value of ESPNOW data
    int         x_axis;             // Joystick x-position
    int         y_axis;             // Joystick y-position
    bool        nav_bttn;           // Joystick push button
    uint8_t     motor1_rpm_pcm;     // PWMs for 4 DC motors
    uint8_t     motor2_rpm_pcm;
    uint8_t     motor3_rpm_pcm;
    uint8_t     motor4_rpm_pcm;
} __attribute__((packed)) sensors_data_t;

// MAC address of receiver. For one-to-many broadcast, change MAC address to FF:FF...:FF
static uint8_t receiver_mac[ESP_NOW_ETH_ALEN]   = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot

static esp_now_peer_info_t peerInfo;                // ESP-NOW pointer holding info about devices (peers)
static sensors_data_t *buf, buffer;                 // Pointer to the struct buffer holding data being sent.
static const char *TAG = "Remote Controller";
static int x = 0, y = 0;

/* WiFi is required to run ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );      // Keep configurations in RAM
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));             // Do not change WiFi device mode
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));   // Both sender & receiver must be on the same channel
}

// Call-back for the event when data is being received
void onDataReceived (uint8_t *mac_addr, uint8_t *data, uint8_t data_len) {

    buf = (sensors_data_t*)data;                            // Allocate memory for buffer to store data being received
    ESP_LOGW(TAG, "Data was received");
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->x_axis);
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->y_axis);
    ESP_LOGI(TAG, "PCM 1: 0x%04x", buf->motor1_rpm_pcm);
}

// Call-back for the event when data is being sent
void onDataSent (uint8_t *mac_addr, esp_now_send_status_t status) {
    //ESP_LOGW(TAG, "Packet send status: 0x%04X", status);
}

// Function to delete peer (i.e. when communication error occurs)
void deletePeer (void) {
    uint8_t delStatus = esp_now_del_peer(receiver_mac);
    if (delStatus != 0) {
        ESP_LOGE("ESP-NOW", "Could not delete peer");
    }
}
// Function for sending the data to the receiver
void sendData (void) {

    buffer.crc = 0;
    buffer.x_axis = 240;
    buffer.y_axis = 256;
    buffer.nav_bttn = 0;
    buffer.motor1_rpm_pcm = 10;
    buffer.motor2_rpm_pcm = 0;
    buffer.motor3_rpm_pcm = 0;
    buffer.motor4_rpm_pcm = 0;

    get_joystick_xy(&x, &y);
    //ESP_LOGI("(x, y)", "[ %d, %d ]", x, y);
    buffer.x_axis = x;
    buffer.y_axis = y;

    // Display brief summary of data being sent.
    //ESP_LOGI(TAG, "Joystick (x,y) position ( %d, %d )", buffer.x_axis, buffer.y_axis);  
    //ESP_LOGI(TAG, "pcm 1, pcm 2 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor1_rpm_pcm, (uint8_t)buffer.motor2_rpm_pcm);
    //ESP_LOGI(TAG, "pcm 3, pcm 4 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor3_rpm_pcm, (uint8_t)buffer.motor4_rpm_pcm);

    // Call ESP-NOW function to send data (MAC address of receiver, pointer to the memory holding data & data length)
    uint8_t result = esp_now_send(receiver_mac, &buffer, sizeof(buffer));

    // If status is NOT OK, display error message and error code (in hexadecimal).
    if (result != 0) {
        ESP_LOGE("ESP-NOW", "Error sending data! Error code: 0x%04X", result);
        deletePeer();
    }
}

// Continous, periodic task that sends data.
static void rc_send_data_task (void *arg) {

    while (true) {
        if (esp_now_is_peer_exist(receiver_mac))
            sendData();
        vTaskDelay (10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    joystick_adc_init();    

    // Initialize NVS to store Wi-Fi configurations
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // ESP-NOW
    wifi_init();                                    // Initialize Wi-Fi
    esp_now_init();                                 // Call ESP-NOW initialization function
    esp_now_register_recv_cb(onDataReceived);       // Define call back for the event when data is being received
    esp_now_register_send_cb(onDataSent);           // Define call back for the event when data is sent received

    // Set ESP-NOW receiver peer configuration values
    memcpy (peerInfo.peer_addr, receiver_mac, 6);   // Copy receiver MAC address
    peerInfo.channel = 1;                           // Define communication channel
    peerInfo.encrypt = false;                       // Keep data unencrypted
    esp_now_add_peer(&peerInfo);                    // Add peer to the list of registered devices

    // Define a task to periodically call function that sends data
    xTaskCreate (rc_send_data_task, "RC", 2048, NULL, 15, NULL);
    xTaskCreate (joystick_task, "RC", 2048, NULL, 2, NULL);
}
