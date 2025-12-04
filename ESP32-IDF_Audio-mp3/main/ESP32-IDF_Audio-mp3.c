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
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD"

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

void app_main(void)
{

}
