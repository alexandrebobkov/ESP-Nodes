/*
 * Internet stream player.
 *
 * Author:          Alexander Bobkov
 * Date Created:    Dec 3, 2025
 * Date Updated:    Dec 3, 2025
 *
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#define MINIMP3_IMPLEMENTATION
#include "minimp3/minimp3.h"
#include "minimp3/minimp3_ex.h"

static const char *TAG = "MP3_STREAM";

// WiFi Configuration
#define WIFI_SSID "IoT_bots"
#define WIFI_PASS "208208208"
#define WIFI_MAXIMUM_RETRY  5

// Audio Stream URL (example: MP3 stream)
#define AUDIO_STREAM_URL "http://jking.cdnstream1.com/b75154_128mp3"
#define MP3_STREAM_URL  "http://jking.cdnstream1.com/b75154_128mp3"

// I2S Configuration for ESP32-C3 with PCM5100
#define I2S_NUM         I2S_NUM_0
#define I2S_BCK_IO      GPIO_NUM_4   // BCK (Bit Clock) -> PCM5100 BCK pin
#define I2S_WS_IO       GPIO_NUM_5   // LRCK (Word Select) -> PCM5100 LRCK pin
#define I2S_DO_IO       GPIO_NUM_6   // DIN (Data) -> PCM5100 DIN pin
#define I2S_SAMPLE_RATE 44100
#define I2S_BUFFER_SIZE 2048

// Stream buffer
#define STREAM_BUFFER_SIZE (32 * 1024)
static uint8_t *stream_buffer = NULL;
static size_t stream_buffer_pos = 0;
static size_t stream_buffer_fill = 0;

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static i2s_chan_handle_t tx_handle = NULL;
static mp3dec_t mp3d;

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize WiFi
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// Initialize I2S
esp_err_t i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    // Enable internal DAC mode bits
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    ESP_LOGI(TAG, "I2S initialized successfully");
    return ESP_OK;
}

// HTTP event handler
esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP Connected");
            break;
        case HTTP_EVENT_HEADERS_SENT:
            ESP_LOGI(TAG, "HTTP Headers sent");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "Header: %s: %s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "Received %d bytes of data, buffer fill: %d", evt->data_len, stream_buffer_fill);
            if (stream_buffer != NULL && stream_buffer_fill + evt->data_len <= STREAM_BUFFER_SIZE) {
                memcpy(stream_buffer + stream_buffer_fill, evt->data, evt->data_len);
                stream_buffer_fill += evt->data_len;
            } else {
                ESP_LOGW(TAG, "Buffer full or NULL, dropping data");
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP Finished");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP Disconnected");
            break;
        default:
            break;
    }
    return ESP_OK;
}

// Decode and play MP3
void decode_and_play_task(void *pvParameters)
{
    mp3dec_frame_info_t info;
    int16_t *pcm = NULL;
    size_t bytes_written;
    
    ESP_LOGI(TAG, "Decode and play task started");
    
    // Allocate PCM buffer on heap instead of stack
    pcm = (int16_t *)malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t));
    if (pcm == NULL) {
        ESP_LOGE(TAG, "Failed to allocate PCM buffer");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "PCM buffer allocated");
    
    // Wait for buffer to be allocated
    while (stream_buffer == NULL) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Stream buffer ready");

    while (1) {
        if (stream_buffer != NULL && stream_buffer_fill - stream_buffer_pos > 1024) {
            int samples = mp3dec_decode_frame(&mp3d, 
                                             stream_buffer + stream_buffer_pos,
                                             stream_buffer_fill - stream_buffer_pos,
                                             pcm, &info);
            
            if (samples > 0) {
                ESP_LOGI(TAG, "Decoded %d samples, %d Hz, %d channels, bitrate: %d", 
                         samples, info.hz, info.channels, info.bitrate_kbps);
                
                // Write PCM data to I2S (samples * channels * bytes_per_sample)
                size_t bytes_to_write = samples * info.channels * sizeof(int16_t);
                esp_err_t ret = i2s_channel_write(tx_handle, pcm, bytes_to_write, 
                                                  &bytes_written, portMAX_DELAY);
                
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
                } else {
                    ESP_LOGD(TAG, "Wrote %d bytes to I2S", bytes_written);
                }
                
                stream_buffer_pos += info.frame_bytes;
                
                // Reset buffer when consumed
                if (stream_buffer != NULL && stream_buffer_pos >= stream_buffer_fill / 2) {
                    size_t remaining = stream_buffer_fill - stream_buffer_pos;
                    if (remaining > 0) {
                        memmove(stream_buffer, 
                               stream_buffer + stream_buffer_pos,
                               remaining);
                    }
                    stream_buffer_fill -= stream_buffer_pos;
                    stream_buffer_pos = 0;
                }
            } else if (samples == 0) {
                ESP_LOGW(TAG, "No samples decoded, skipping frame");
                stream_buffer_pos += 1;  // Skip one byte and try again
            } else {
                ESP_LOGE(TAG, "Decode error: %d", samples);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        } else {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
    
    // Cleanup (never reached but good practice)
    free(pcm);
}

// Stream MP3 from internet
void stream_mp3_task(void *pvParameters)
{
    esp_http_client_config_t config = {
        .url = MP3_STREAM_URL,
        .event_handler = http_event_handler,
        .buffer_size = 4096,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    ESP_LOGI(TAG, "Starting MP3 stream from: %s", MP3_STREAM_URL);
    
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP Stream completed");
    } else {
        ESP_LOGE(TAG, "HTTP Stream failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "ESP32-C3 MP3 Streamer Starting...");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    // Allocate stream buffer
    stream_buffer = (uint8_t *)malloc(STREAM_BUFFER_SIZE);
    if (stream_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate stream buffer");
        return;
    }
    ESP_LOGI(TAG, "Stream buffer allocated: %d bytes", STREAM_BUFFER_SIZE);

    // Initialize MP3 decoder
    mp3dec_init(&mp3d);
    ESP_LOGI(TAG, "MP3 decoder initialized");

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();
    ESP_LOGI(TAG, "WiFi connected successfully");

    // Initialize I2S
    ESP_LOGI(TAG, "Initializing I2S...");
    ret = i2s_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2S initialized successfully");

    // Create tasks
    ESP_LOGI(TAG, "Creating tasks...");
    BaseType_t task_ret;
    
    task_ret = xTaskCreate(decode_and_play_task, "decode_play", 8192, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create decode_and_play_task");
        return;
    }
    ESP_LOGI(TAG, "Decode task created");
    
    task_ret = xTaskCreate(stream_mp3_task, "stream_mp3", 8192, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create stream_mp3_task");
        return;
    }
    ESP_LOGI(TAG, "Stream task created");

    ESP_LOGI(TAG, "All tasks started successfully!");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", esp_get_free_heap_size());
    
    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Status - Buffer fill: %d bytes, Free heap: %lu", 
                 stream_buffer_fill, esp_get_free_heap_size());
    }
}