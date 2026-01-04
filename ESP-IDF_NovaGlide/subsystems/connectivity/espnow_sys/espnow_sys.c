#include "espnow_sys.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ESPNOW_SYS";
static espnow_system_t *g_sys = NULL;

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data,
                           int len)
{
    if (!g_sys || len <= 0 || len > sizeof(sensors_data_t)) {
        ESP_LOGW(TAG, "Invalid data: len=%d (expected %d)", len, sizeof(sensors_data_t));
        return;
    }

    memcpy(&g_sys->last_data, data, len);
    g_sys->last_len = len;

    // Enhanced logging
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP-NOW DATA RECEIVED");
    ESP_LOGI(TAG, "X-axis (rc_x): %d", g_sys->last_data.x_axis);
    ESP_LOGI(TAG, "Y-axis (rc_y): %d", g_sys->last_data.y_axis);
    ESP_LOGI(TAG, "M1: %d, M2: %d, M3: %d, M4: %d",
             g_sys->last_data.motor1_rpm_pcm,
             g_sys->last_data.motor2_rpm_pcm,
             g_sys->last_data.motor3_rpm_pcm,
             g_sys->last_data.motor4_rpm_pcm);
    ESP_LOGI(TAG, "Data length: %d bytes", len);
    ESP_LOGI(TAG, "========================================");
}

static void espnow_send_impl(espnow_system_t *self,
                             const uint8_t *data,
                             int len)
{
    if (!data || len <= 0) return;
    esp_now_send(NULL, data, len);
}

static void espnow_update_impl(espnow_system_t *self, TickType_t now)
{
    (void)self;
    (void)now;
}

void espnow_system_init(espnow_system_t *sys)
{
    memset(&sys->last_data, 0, sizeof(sys->last_data));
    sys->last_len = 0;
    sys->send = espnow_send_impl;
    sys->update = espnow_update_impl;

    g_sys = sys;

    ESP_LOGI(TAG, "Initializing ESP-NOW...");

    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ESP-NOW init success");

    ret = esp_now_register_recv_cb(espnow_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW register callback failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ESP-NOW callback registered");

    // Print MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGW(TAG, "Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI(TAG, "ESP-NOW initialized - waiting for data from transmitter...");
    ESP_LOGI(TAG, "Expected data size: %d bytes", sizeof(sensors_data_t));
}
