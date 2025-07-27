#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_now.h"

#include "sensors_data.h"
#include "config.h"

static sensors_data_t buf;

/* WiFi should start before using ESPNOW */
void wifi_init()
{
    /*
    * STAND-ALONE ESP-NOW
    */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    //ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));//ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK( esp_wifi_start());
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE));
    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    #endif

    /*
    * WI-FI
    */
    /*ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));//ESPNOW_WIFI_MODE));
        wifi_config_t wifi_config = {
        .sta = {
            .ssid = "IoT_bots",
            .password = "208208208",
            .channel = 11,
        },
    };
    ESP_ERROR_CHECK (esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_connect());*/
}

void transmission_init() {

    esp_err_t espnow_ret = esp_now_init();
    if (espnow_ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init() failed: %s", esp_err_to_name(espnow_ret));
        return;
    }
    ESP_LOGI(TAG, "ESPNOW initialized successfully");
}

void onDataReceived (const uint8_t *mac_addr, const uint8_t *data, uint8_t data_len) {

    ESP_LOGI(TAG, "Data received from: %02x:%02x:%02x:%02x:%02x:%02x, len=%d", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], data_len);

    memcpy(&buf, data, sizeof(buf));
}