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

#include "commands.h"

esp_console_config_t *console_config;
esp_console_cmd_t *cmd1;
esp_console_cmd_func_t *cmd1_func; 

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
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    initialize_nvs();

    esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    repl_config.prompt = "foxie >";

    /*
        INITIALIZE COMMANDS
    */
   register_commands();
   
   /*
        START CONSOLE REPL
   */
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

}
