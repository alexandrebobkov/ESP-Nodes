/*
    Dcription:  Console interface
    by:         Alexander Bobkov
    Created:    Jan 9, 2025

*/

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_flash.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "esp_console.h"

esp_console_config_t *console_config;
esp_console_cmd_t *cmd1;

static void initialize_nvs (void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK (nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    initialize_nvs();

    /*
        INITIALIZE COMMANDS
    */
   esp_console_init(console_config);
   //esp_console_register_help_command();

}
