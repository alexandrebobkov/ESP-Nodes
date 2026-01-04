#include "espnow_sys.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ESPNOW_SYS";
static espnow_system_t *g_sys = NULL;

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data,
                           int len)
{
    if (!g_sys || len <= 0 || len > sizeof(sensors_data_t)) return;

    memcpy(&g_sys->last_data, data, len);
    g_sys->last_len = len;

    ESP_LOGI(TAG, "Received %d bytes - X:%d Y:%d", len,
             g_sys->last_data.x_axis, g_sys->last_data.y_axis);
}

static void espnow_send_impl(espnow_system_t *self,
                             const uint8_t *data,
                             int len)
{
    if (!data || len <= 0) return;
    esp_now_send(NULL, data, len);  // NULL = broadcast
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

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "ESP-NOW initialized");
}
