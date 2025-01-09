/*
    Dcription:  Console interface
    by:         Alexander Bobkov
    Created:    Jan 9, 2025

*/

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_console.h"

static void initialize_nvs (void) {
    esp_err_t err = nvs_flash_init();
    if (err = ESP_ERR_NVS_NO_FREE_PAGES || err = ESP_ERR_NVS_NEW_VERSION) {
        ESP_ERROR_CHECK (nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{

}
