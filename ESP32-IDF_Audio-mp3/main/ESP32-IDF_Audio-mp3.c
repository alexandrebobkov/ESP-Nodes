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

void i2s_init(void) {
    // 1. I2S Configuration
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX, // Master mode, Transmitter
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, // Standard I2S format
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
        .dma_buf_count = DMA_BUF_COUNT,
        .dma_buf_len = DMA_BUF_LEN,
        .use_apll = false, // ESP32-C3 does not have APLL, use default clock
        .tx_desc_auto_clear = true, // Clear TX DMA descriptors on underflow
    };


    // 2. I2S Pin Configuration
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE, // Not always needed in master mode
        .bck_io_num = I2S_BCK_IO_NUM,
        .ws_io_num = I2S_WS_IO_NUM,
        .data_out_num = I2S_DOUT_IO_NUM,
        .data_in_num = I2S_PIN_NO_CHANGE // Not used for playback
    };

    // 3. Install and set pins
    ESP_LOGI(TAG, "Initializing I2S driver...");
    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pin_config));
    ESP_LOGI(TAG, "I2S driver initialized.");
}

void audio_player_task(void *pvParameter) {
    // 1. HTTP/Network Setup (Simplified placeholder)
    // In a real application, you would use esp_http_client_get()
    // to open a connection to the internet radio URL (e.g., a .mp3 or .aac stream).
    // The streaming URL would be passed as a parameter or defined globally.
    // const char *stream_url = "http://internet.radio.stream:port/stream";

    // 2. Main Stream Loop
    while (1) {
        // --- Step A: Get Raw Audio Data ---
        // * Perform HTTP GET request for audio data chunks.
        // * Pass the received data to an audio decoder (MP3, AAC, etc.).
        // * The decoder output is raw PCM audio data (e.g., 16-bit stereo samples).

        // This is a dummy buffer for raw 16-bit stereo PCM data
        // Buffer size should match your DMA buffer configuration
        int16_t pcm_data[DMA_BUF_COUNT * DMA_BUF_LEN / sizeof(int16_t)];
        size_t pcm_data_size = sizeof(pcm_data);
        size_t bytes_written;

        // In the real code, 'decode_audio_stream_to_pcm(pcm_data, pcm_data_size)'
        // would replace the dummy delay below.

        // --- Simulated Data for testing the I2S setup ---
        // Dummy block to simulate a decoder providing PCM data
        // vTaskDelay(pdMS_TO_TICKS(100)); // Simulate decode and fetch time
        // for(size_t i = 0; i < pcm_data_size / sizeof(int16_t); i++) {
        //     // Generate a simple sine wave or fill with zeros
        //     pcm_data[i] = (i % 200) < 100 ? 5000 : -5000;
        // }


        // --- Step B: Write to I2S ---
        if (/* decoder successfully provided pcm_data */ 1) {
            // Write the raw PCM data to the I2S port
            esp_err_t err = i2s_write(I2S_PORT, pcm_data, pcm_data_size, &bytes_written, portMAX_DELAY);

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "I2S Write failed: %s", esp_err_to_name(err));
            } else if (bytes_written != pcm_data_size) {
                ESP_LOGW(TAG, "I2S wrote %d bytes, expected %d", bytes_written, pcm_data_size);
            }
        } else {
            // Handle stream end, error, or buffer underflow
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void app_main(void)
{

}
