/*
    Dcription:  Console interface
    by:         Alexander Bobkov
    Created:    Jan 9, 2025

*/

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_flash.h"
#include "sdkconfig.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "esp_console.h"

#include "commands.h"
#include "cmd_gpio_set.h"
#include "cmd_gpio_get.h"
#include "cmd_scan_wifi.h"

esp_console_config_t *console_config;
esp_console_cmd_t *cmd1;
esp_console_cmd_func_t *cmd1_func; 

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE("Console", "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}

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
    printf("\n============================================\n");
    printf("Welcome to Breadboard Power Adapter ESP32-C3\n");
    printf("\n============================================\n");

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    
    initialize_nvs();
    //initialize_filesystem();
    //repl_config.history_save_path = HISTORY_PATH;

    /*
        INITIALIZE COMMANDS
    */
    //register_commands();

    repl_config.prompt = "zen >";
    
    // Update settings in menu config
    //esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();  
    //ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));  
    esp_console_dev_usb_serial_jtag_config_t usb_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usb_config, &repl_config, &repl));
    

    esp_console_register_help_command();
    register_commands();
    register_gpio_set_cmd();
    register_gpio_get_cmd();
    register_scan_wifi_cmd();
    /*
        START CONSOLE REPL
    */
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

}
