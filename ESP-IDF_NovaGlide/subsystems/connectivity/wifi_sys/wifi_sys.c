#include "wifi_sys.h"
#include "esp_log.h"
#include "esp_netif.h"

static const char *TAG = "WIFI_SYS";

void wifi_system_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Give WiFi time to connect
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Log connection info
        uint8_t channel;
        wifi_second_chan_t second;
        esp_wifi_get_channel(&channel, &second);

        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);

        ESP_LOGI(TAG, "WiFi initialized and connecting...");
        ESP_LOGI(TAG, "SSID: %s", WIFI_SSID);
        ESP_LOGW(TAG, "WiFi Channel: %d", channel);
        ESP_LOGW(TAG, "Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        ESP_LOGW(TAG, "Transmitter will scan channels to find receiver");
}
