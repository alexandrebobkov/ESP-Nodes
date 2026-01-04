#include "system_init.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

static const char *TAG = "SYSTEM_INIT";

void system_init(void)
{
    // --- NVS ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // --- Event loop ---
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // --- Network stack ---
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_LOGI(TAG, "System initialization complete");
}
