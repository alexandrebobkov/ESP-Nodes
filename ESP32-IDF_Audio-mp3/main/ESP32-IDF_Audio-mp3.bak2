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
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

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
#define STREAM_BUFFER_SIZE (64 * 1024)
static uint8_t *stream_buffer = NULL;
static size_t stream_buffer_pos = 0;
static size_t stream_buffer_fill = 0;
#define AUDIO_QUEUE_SIZE 10

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static i2s_chan_handle_t tx_handle = NULL;
static HMP3Decoder hMP3Decoder = NULL;

typedef struct {
    uint8_t *data;
    size_t len;
} audio_frame_t;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static i2s_chan_handle_t tx_handle = NULL;
static QueueHandle_t audio_queue = NULL;
static bool stream_running = false;

// WiFi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
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
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_SSID, WIFI_PASS);
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
        .slot_cfg = I2S_STD_STEREO_SLOTS_DEFAULT_CONFIG(I2S_BITS_PER_SAMPLE, I2S_SLOT_MODE_STEREO),
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

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    ESP_LOGI(TAG, "I2S initialized - BCK:%d WS:%d DOUT:%d @ %dHz", 
             I2S_BCK_IO, I2S_WS_IO, I2S_DO_IO, I2S_SAMPLE_RATE);
    
    return ESP_OK;
}

// HTTP Event Handler
esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        if (!stream_running) break;
        
        // Queue audio data for playback
        if (evt->data_len > 0) {
            audio_frame_t frame = {
                .data = (uint8_t *)malloc(evt->data_len),
                .len = evt->data_len
            };
            if (frame.data) {
                memcpy(frame.data, evt->data, evt->data_len);
                if (xQueueSend(audio_queue, &frame, 0) != pdPASS) {
                    ESP_LOGW(TAG, "Audio queue full, dropping frame");
                    free(frame.data);
                }
            }
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        stream_running = false;
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

// Stream MP3 from internet
void stream_mp3_task(void *pvParameters)
{
    esp_http_client_config_t config = {
        .url = MP3_STREAM_URL,
        .event_handler = http_event_handler,
        .buffer_size = 4096,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    ESP_LOGI(TAG, "Starting MP3 stream from: %s", MP3_STREAM_URL);
    stream_running = true;
    
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP Stream completed, Status: %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    stream_running = false;
    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}

// Play audio from queue
void audio_playback_task(void *pvParameters)
{
    audio_frame_t frame;
    size_t bytes_written;

    ESP_LOGI(TAG, "Audio playback task started");

    while (1) {
        if (xQueueReceive(audio_queue, &frame, portMAX_DELAY) == pdPASS) {
            // Write to I2S
            esp_err_t err = i2s_channel_write(tx_handle, frame.data, frame.len, &bytes_written, portMAX_DELAY);
            
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "I2S write error: %s", esp_err_to_name(err));
            }
            
            free(frame.data);
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP32-C3 MP3 Streamer");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    // Create audio queue
    audio_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_frame_t));
    if (audio_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create audio queue");
        return;
    }

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    // Initialize I2S
    ESP_LOGI(TAG, "Initializing I2S...");
    i2s_init();

    // Create tasks
    xTaskCreate(audio_playback_task, "audio_play", 4096, NULL, 10, NULL);
    xTaskCreate(stream_mp3_task, "stream_mp3", 4096, NULL, 5, NULL);
}