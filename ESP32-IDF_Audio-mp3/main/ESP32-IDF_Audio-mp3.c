/*
 * Internet stream player.
 *
 * Author:          Alexander Bobkov
 * Date Created:    Dec 3, 2025
 * Date Updated:    Dec 6, 2025
 *
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

// Third-party MP3 decoder (minimp3)
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#define MINIMP3_IMPLEMENTATION
#include "minimp3/minimp3.h"
#include "minimp3/minimp3_ex.h"

static const char *TAG = "MP3_STREAM";

// WiFi Configuration
#define WIFI_SSID      "activcount"
#define WIFI_PASS      "l0ve2c0unt"
#define WIFI_MAXIMUM_RETRY  5

// MP3 Stream URL
#define MP3_STREAM_URL "http://jking.cdnstream1.com/b75154_128mp3"

// I2S Configuration for ESP32-C3 with PCM5100
#define I2S_NUM         I2S_NUM_0
#define I2S_BCK_IO      GPIO_NUM_4   // BCK (Bit Clock) -> PCM5100 BCK pin
#define I2S_WS_IO       GPIO_NUM_5   // LRCK (Word Select) -> PCM5100 LRCK pin
#define I2S_DO_IO       GPIO_NUM_6   // DIN (Data) -> PCM5100 DIN pin
#define I2S_SAMPLE_RATE 44100
#define I2S_BUFFER_SIZE 2048

// Stream buffer - increase for smoother playback
#define STREAM_BUFFER_SIZE (32 * 1024)  // 32KB buffer for better buffering
static uint8_t *stream_buffer = NULL;
static size_t stream_buffer_pos = 0;
static size_t stream_buffer_fill = 0;
static bool stream_paused = false;  // Pause flag to control streaming

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
    chan_cfg.auto_clear = true;  // Auto clear DMA buffer
    chan_cfg.dma_desc_num = 16;  // Increase DMA descriptors for smoother playback
    chan_cfg.dma_frame_num = 240;  // Increase frame size
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
    
    // Configure for best quality
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    ESP_LOGI(TAG, "I2S initialized successfully");
    ESP_LOGI(TAG, "I2S Pins - BCK: GPIO%d, WS: GPIO%d, DOUT: GPIO%d", I2S_BCK_IO, I2S_WS_IO, I2S_DO_IO);
    ESP_LOGI(TAG, "I2S Config - Sample Rate: %d Hz, Bits: 16, Channels: Stereo", I2S_SAMPLE_RATE);
    
    // Test I2S by sending silent data
    int16_t test_data[128] = {0};
    size_t written;
    esp_err_t ret = i2s_channel_write(tx_handle, test_data, sizeof(test_data), &written, 1000);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2S test write successful: %d bytes written", written);
    } else {
        ESP_LOGE(TAG, "I2S test write failed: %s", esp_err_to_name(ret));
    }
    
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
            ESP_LOGD(TAG, "Header: %s: %s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            // Add data to buffer with flow control
            if (stream_buffer != NULL && !stream_paused) {
                if (stream_buffer_fill + evt->data_len <= STREAM_BUFFER_SIZE) {
                    memcpy(stream_buffer + stream_buffer_fill, evt->data, evt->data_len);
                    stream_buffer_fill += evt->data_len;
                    ESP_LOGD(TAG, "Received %d bytes, buffer: %d/%d", 
                            evt->data_len, stream_buffer_fill, STREAM_BUFFER_SIZE);
                } else {
                    // Buffer almost full, slow down
                    ESP_LOGD(TAG, "Buffer full, pausing...");
                    vTaskDelay(5 / portTICK_PERIOD_MS);
                }
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
    uint32_t frames_decoded = 0;
    uint32_t total_bytes_written = 0;
    bool first_frame = true;
    
    ESP_LOGI(TAG, "Decode and play task started, free heap: %lu", esp_get_free_heap_size());
    
    // Allocate PCM buffer
    pcm = (int16_t *)heap_caps_malloc(4608 * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (pcm == NULL) {
        ESP_LOGE(TAG, "Failed to allocate PCM buffer");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "PCM buffer allocated, free heap: %lu", esp_get_free_heap_size());
    
    // Wait for buffer to be allocated and fill with some data before starting
    while (stream_buffer == NULL) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Wait for buffer to fill up a bit before starting playback
    ESP_LOGI(TAG, "Waiting for initial buffer fill...");
    while (stream_buffer_fill < 8192) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Stream buffer ready with %d bytes, starting playback", stream_buffer_fill);

    while (1) {
        // Make sure we have enough data in buffer before decoding
        if (stream_buffer != NULL && stream_buffer_fill - stream_buffer_pos > 2048) {
            int samples = mp3dec_decode_frame(&mp3d, 
                                             stream_buffer + stream_buffer_pos,
                                             stream_buffer_fill - stream_buffer_pos,
                                             pcm, &info);
            
            if (samples > 0) {
                frames_decoded++;
                
                // Log first frame info only
                if (first_frame) {
                    ESP_LOGI(TAG, "First frame decoded: %d samples, %d Hz, %d channels, %d kbps", 
                             samples, info.hz, info.channels, info.bitrate_kbps);
                    first_frame = false;
                }
                
                // Write PCM data to I2S - blocking write for smooth playback
                size_t bytes_to_write = samples * info.channels * sizeof(int16_t);
                esp_err_t ret = i2s_channel_write(tx_handle, pcm, bytes_to_write, 
                                                  &bytes_written, portMAX_DELAY);
                
                if (ret == ESP_OK) {
                    total_bytes_written += bytes_written;
                } else {
                    ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
                }
                
                // Log every 500 frames (reduce logging overhead)
                if (frames_decoded % 500 == 0) {
                    ESP_LOGI(TAG, "Playing: Frame %lu | Buffer: %d bytes | Free heap: %lu", 
                             frames_decoded, stream_buffer_fill, esp_get_free_heap_size());
                }
                
                stream_buffer_pos += info.frame_bytes;
                
                // Reset buffer when half consumed
                if (stream_buffer != NULL && stream_buffer_pos >= STREAM_BUFFER_SIZE / 2) {
                    size_t remaining = stream_buffer_fill - stream_buffer_pos;
                    if (remaining > 0) {
                        memmove(stream_buffer, 
                               stream_buffer + stream_buffer_pos,
                               remaining);
                    }
                    stream_buffer_fill = remaining;
                    stream_buffer_pos = 0;
                }
            } else if (samples == 0) {
                // Skip byte and retry
                stream_buffer_pos += 1;
            } else {
                // Decode error - skip byte
                stream_buffer_pos += 1;
            }
        } else {
            // Not enough data, wait for more
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    
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

    // Create tasks with very large stack for MP3 decoder (it needs ~20KB internally)
    ESP_LOGI(TAG, "Creating tasks...");
    BaseType_t task_ret;
    
    // Increase to 24KB and higher priority (priority 10) - minimp3 uses large internal buffers on stack
    task_ret = xTaskCreate(decode_and_play_task, "decode_play", 24576, NULL, 10, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create decode_and_play_task");
        return;
    }
    ESP_LOGI(TAG, "Decode task created with 24KB stack, priority 10");
    
    task_ret = xTaskCreate(stream_mp3_task, "stream_mp3", 4096, NULL, 3, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create stream_mp3_task");
        return;
    }
    ESP_LOGI(TAG, "Stream task created with priority 3");

    ESP_LOGI(TAG, "All tasks started successfully!");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", esp_get_free_heap_size());
    
    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Status - Buffer fill: %d bytes, Free heap: %lu", 
                 stream_buffer_fill, esp_get_free_heap_size());
    }
}