#include "espnow_sys.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ESPNOW_SYS";

// -----------------------------------------------------------------------------
// ESP-NOW receive callback
// -----------------------------------------------------------------------------
static espnow_system_t *g_sys = NULL;

static void espnow_recv_cb(const uint8_t *mac_addr,
                           const uint8_t *data,
                           int len)
{
    if (!g_sys || len <= 0 || len > 32) return;

    memcpy(g_sys->last_data, data, len);
    g_sys->last_len = len;

    ESP_LOGI(TAG, "Received %d bytes via ESP-NOW", len);
}

// -----------------------------------------------------------------------------
// Send implementation
// -----------------------------------------------------------------------------
static void espnow_send_impl(espnow_system_t *self,
                             const uint8_t *data,
                             int len)
{
    if (!data || len <= 0) return;

    esp_now_send(NULL, data, len);  // NULL = broadcast
}

// -----------------------------------------------------------------------------
// Update implementation (scheduler-driven)
// -----------------------------------------------------------------------------
static void espnow_update_impl(espnow_system_t *self, TickType_t now)
{
    (void)self;
    (void)now;
    // Nothing periodic yet — but this hook is ready for future logic
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------
void espnow_system_init(espnow_system_t *sys)
{
    memset(sys->last_data, 0, sizeof(sys->last_data));
    sys->last_len = 0;

    sys->send   = espnow_send_impl;
    sys->update = espnow_update_impl;

    g_sys = sys;

    // Wi-Fi must be initialized before ESP-NOW
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Init ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "ESP-NOW initialized");
}
