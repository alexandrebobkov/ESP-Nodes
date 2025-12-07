/*
 * Internet stream player using libhelix MP3 decoder.
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

// Helix MP3 decoder
#include "mp3dec.h"

static const char *TAG = "MP3_STREAM";

// WiFi Configuration
#define WIFI_SSID      "IoT_bots"
#define WIFI_PASS      "208208208"
#define WIFI_MAXIMUM_RETRY  5

// MP3 Stream URL
#define MP3_STREAM_URL "http://jking.cdnstream1.com/b75154_128mp3"

// I2S Configuration for ESP32-C3 with PCM5100
#define I2S_NUM         I2S_NUM_0
#define I2S_BCK_IO      GPIO_NUM_4
#define I2S_WS_IO       GPIO_NUM_5
#define I2S_DO_IO       GPIO_NUM_6
#define I2S_SAMPLE_RATE 44100

// Helix decoder constants
#define MAINBUF_SIZE    (1940)  // Size of main buffer for MP3 data
#define MAX_NGRAN       2
#define MAX_NCHAN       2
#define MAX_NSAMP       (MAX_NGRAN * MAX_NCHAN * 576)  // Max samples per frame

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
static HMP3Decoder hMP3Decoder = NULL;

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
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 240;
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
    
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    ESP_LOGI(TAG, "I2S initialized - BCK:%d WS:%d DOUT:%d @ %dHz", 
             I2S_BCK_IO, I2S_WS_IO, I2S_DO_IO, I2S_SAMPLE_RATE);
    
    return ESP_OK;
}

// HTTP event handler
esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP Connected");
            break;
        case HTTP_EVENT_ON_DATA:
            if (stream_buffer != NULL && stream_buffer_fill + evt->data_len <= STREAM_BUFFER_SIZE) {
                memcpy(stream_buffer + stream_buffer_fill, evt->data, evt->data_len);
                stream_buffer_fill += evt->data_len;
            }
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP Disconnected");
            break;
        default:
            break;
    }
    return ESP_OK;
}

// Decode and play MP3 using Helix
void decode_and_play_task(void *pvParameters)
{
    int err, offset, bytes_left;
    unsigned char *read_ptr;
    short *pcm_buffer = NULL;
    MP3FrameInfo mp3_frame_info;
    size_t bytes_written;
    uint32_t frames_decoded = 0;
    bool first_frame = true;
    
    ESP_LOGI(TAG, "Decode task started, free heap: %lu", esp_get_free_heap_size());
    
    // Allocate PCM buffer
    pcm_buffer = (short *)heap_caps_malloc(MAX_NSAMP * sizeof(short), MALLOC_CAP_8BIT);
    if (pcm_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate PCM buffer");
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize Helix decoder
    hMP3Decoder = MP3InitDecoder();
    if (hMP3Decoder == NULL) {
        ESP_LOGE(TAG, "Failed to initialize Helix decoder");
        free(pcm_buffer);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Helix decoder initialized, free heap: %lu", esp_get_free_heap_size());
    
    // Wait for buffer
    while (stream_buffer == NULL) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Wait for initial buffer fill
    ESP_LOGI(TAG, "Waiting for buffer fill...");
    while (stream_buffer_fill < 16384) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Buffer ready (%d bytes), starting playback", stream_buffer_fill);

    while (1) {
        bytes_left = stream_buffer_fill - stream_buffer_pos;
        
        if (bytes_left >= MAINBUF_SIZE) {
            read_ptr = stream_buffer + stream_buffer_pos;
            
            // Find sync word
            offset = MP3FindSyncWord(read_ptr, bytes_left);
            if (offset < 0) {
                // No sync found, skip some data
                stream_buffer_pos += (bytes_left > 100) ? 100 : bytes_left;
                continue;
            }
            
            read_ptr += offset;
            bytes_left -= offset;
            stream_buffer_pos += offset;
            
            // Decode frame
            err = MP3Decode(hMP3Decoder, &read_ptr, &bytes_left, pcm_buffer, 0);
            
            if (err == ERR_MP3_NONE) {
                // Get frame info
                MP3GetLastFrameInfo(hMP3Decoder, &mp3_frame_info);
                
                frames_decoded++;
                
                if (first_frame) {
                    ESP_LOGI(TAG, "Decoding: %d Hz, %d ch, %d kbps, %d samples/frame",
                             mp3_frame_info.samprate, mp3_frame_info.nChans,
                             mp3_frame_info.bitrate / 1000, mp3_frame_info.outputSamps);
                    first_frame = false;
                }
                
                // Write to I2S
                size_t bytes_to_write = mp3_frame_info.outputSamps * sizeof(short);
                i2s_channel_write(tx_handle, pcm_buffer, bytes_to_write, &bytes_written, portMAX_DELAY);
                
                // Update position
                stream_buffer_pos = (read_ptr - stream_buffer);
                
                // Manage buffer
                if (stream_buffer_pos >= (STREAM_BUFFER_SIZE * 3 / 4)) {
                    size_t remaining = stream_buffer_fill - stream_buffer_pos;
                    if (remaining > 0) {
                        memmove(stream_buffer, stream_buffer + stream_buffer_pos, remaining);
                    }
                    stream_buffer_fill = remaining;
                    stream_buffer_pos = 0;
                }
            } else {
                // Decode error, skip ahead
                stream_buffer_pos += 1;
            }
        } else {
            // Not enough data
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    
    MP3FreeDecoder(hMP3Decoder);
    free(pcm_buffer);
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
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP32-C3 MP3 Streamer with Helix decoder");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    // Allocate stream buffer
    stream_buffer = (uint8_t *)malloc(STREAM_BUFFER_SIZE);
    if (stream_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate stream buffer");
        return;
    }
    ESP_LOGI(TAG, "Stream buffer allocated: %d bytes", STREAM_BUFFER_SIZE);

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    // Initialize I2S
    ESP_LOGI(TAG, "Initializing I2S...");
    i2s_init();

    // Create tasks
    xTaskCreate(decode_and_play_task, "decode_play", 8192, NULL, 10, NULL);
    xTaskCreate(stream_mp3_task, "stream_mp3", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "Tasks started, free heap: %lu", esp_get_free_heap_size());
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}