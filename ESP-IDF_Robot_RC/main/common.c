#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_now.h"
//#include "esp_netif.h"
//#include "esp_mac.h"

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/*static esp_err_t rc_espnow_init (void) {

    espnow_data_packet_t *send_packet;

    send_packet = malloc(sizeof(espnow_data_packet_t));
    if (send_packet == NULL) {
        ESP_LOGE(TAG, "malloc fail.");
        return ESP_FAIL;
    }

    memset(send_packet, 0, sizeof(espnow_data_packet_t));
    memcpy(send_packet->dest_mac, receiver_mac, ESP_NOW_ETH_ALEN);
    send_packet->len = CONFIG_ESPNOW_SEND_LEN; // 128
    send_packet->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    sensors_data_prepare(send_packet);
    xTaskCreate(rc_send_data_task2, "controller data packets task", 2048, send_packet, 8, NULL);

    return ESP_OK;
}*/