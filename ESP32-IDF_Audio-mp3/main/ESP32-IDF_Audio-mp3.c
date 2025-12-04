/*
 * Internet stream player.
 *
 * Author:          Alexander Bobkov
 * Date Created:    Dec 3, 2025
 * Date Updated:    Dec 3, 2025
 *
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "driver/i2s.h"
#include "nvs_flash.h"

// --- Configuration ---

// Wi-Fi Credentials
#define WIFI_SSID      "IoT_bots"
#define WIFI_PASSWORD  "208208208"

// I2S Configuration (Modify GPIOs for your specific board/DAC)
#define I2S_PORT            I2S_NUM_0
#define I2S_BCK_IO_NUM      GPIO_NUM_7  // Bit Clock (BCK)
#define I2S_WS_IO_NUM       GPIO_NUM_6  // Word Select (WS/LRCK)
#define I2S_DOUT_IO_NUM     GPIO_NUM_5  // Data Out (DIN)

// Audio Stream Configuration (Example for 44.1kHz, 16-bit Stereo)
#define SAMPLE_RATE         44100
#define BITS_PER_SAMPLE     I2S_BITS_PER_SAMPLE_16BIT
#define DMA_BUF_COUNT       8
#define DMA_BUF_LEN         256 // Bytes per DMA buffer

// Task Tag for logging
static const char *TAG = "AUDIO_PLAYER";

// FreeRTOS Event Group for Wi-Fi status
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// --- Function Prototypes ---
void wifi_init_sta(void);
void i2s_init(void);
void audio_player_task(void *pvParameter);

void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Configure and start Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");

    // Wait for connection successful
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi connected.");
}

// Wi-Fi Event Handler
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void app_main(void)
{

}
