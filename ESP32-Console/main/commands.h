#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_chip_info.h"
#include "soc/rtc.h"
#include "esp_console.h"
#include "esp_log.h"

// Display information about ESP32 chip.
static int exec_info_cmd(int argc, char **argv);
static void register_info(void);

// Display GPIOs states.
//static int exec_print_gpio(int argc, char **argv);
//static void register_print_gpio(void);

struct arg_lit *temp, *help;
struct arg_end *end;
static struct {
    struct arg_str *temp;
    struct arg_int *chip_temp;
    struct arg_str *voltage;
    struct arg_str *current;
    struct arg_end *end;
} info_args;
static int exec_info_cmd (int argc, char **argv) {

    rtc_cpu_freq_config_t freq_config;
    rtc_clk_cpu_freq_get_config(&freq_config);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t total_internal_memory = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    uint32_t free_internal_memory = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    ESP_LOGW("CLI", "Information about microcontroller chip.");
    ESP_LOGI("Chip Info", "Source Clock Frequency: %" PRIu32 " MHz", freq_config.source_freq_mhz);
    ESP_LOGI("Chip Info", "Effective CPU Frequency: %" PRIu32 " MHz", freq_config.freq_mhz);
    ESP_LOGI("Memory Info", "Total DRAM (internal memory): %"PRIu32" (KB)", total_internal_memory/1024);
    ESP_LOGI("Memory Info", "Free DRAM (internal memory): %"PRIu32" (KB)", free_internal_memory/1024);

    uint32_t features = chip_info.features;

    char binary_str[33]; // 32 bits + null terminator
    for (int i = 31; i >= 0; i--) {
        binary_str[31 - i] = (features & (1U << i)) ? '1' : '0';
    }
    binary_str[32] = '\0'; // Null terminate the string
    ESP_LOGI("Chip Info", "Features Bitmap: %s", binary_str);

    ESP_LOGI("Chip Info", "Embedded Flash: %s", (features & CHIP_FEATURE_EMB_FLASH) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "Embedded PSRAM: %s", (features & CHIP_FEATURE_EMB_PSRAM) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "Wi-Fi 2.4GHz support: %s", (features & CHIP_FEATURE_WIFI_BGN) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "IEEE 802.15.4 support: %s", (features & CHIP_FEATURE_IEEE802154) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "Bluetooth Classic support: %s", (features & CHIP_FEATURE_BT) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "Bluetooth LE (BLE) support: %s", (features & CHIP_FEATURE_BLE) ? "Yes" : "No");
    //CONFIG_SOC_LIGHT_SLEEP_SUPPORTED
    ESP_LOGI("Chip Info", "Compile date %i", CONFIG_APP_COMPILE_TIME_DATE);
    //CONFIG_APPTRACE_UART_BAUDRATE
    //CONFIG_APPTRACE_UART_RX_GPIO
    //CONFIG_APPTRACE_UART_TX_GPIO
    //CONFIG_BOOTLOADER_COMPILE_TIME_DATE
    //CONFIG_CONSOLE_UART_NUM
    //ESP_LOGI("Chip Info", "Universal MAC address %i", CONFIG_ESP32C3_UNIVERSAL_MAC_ADDRESSES);
    //ESP_LOGI("Chip Info", "Wi-Fi MAC address %i", CONFIG_ESP_MAC_ADDR_UNIVERSE_WIFI_AP);
    ESP_LOGI("Chip Info", "Wi-Fi Power %i", CONFIG_ESP_PHY_MAX_WIFI_TX_POWER);

    int nerrors = arg_parse(argc, argv, (void**) &info_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, info_args.end, argv[0]);
        return 1;
    }

    if (info_args.temp->count != 0) {
        if (strcmp(info_args.temp->sval[0], "C") == 0)
            ESP_LOGI("CLI", "Displaying temperature in Celcius.");
        if (strcmp(info_args.temp->sval[0], "F") == 0)
            ESP_LOGI("CLI", "Displaying temperature in Farenheight.");
    }
    if (info_args.chip_temp->count != 0) {
        ESP_LOGW("CLI", "info(): Chip Temperature: %d", 35);
    }
    if (info_args.voltage->count != 0) {
        if (strcmp(info_args.voltage->sval[0], "mV") == 0)
            ESP_LOGI("CLI", "Displaying voltage, [mV].");
        else if (strcmp(info_args.voltage->sval[0], "V") == 0)
            ESP_LOGI("CLI", "Displaying voltage, [V].");
    }
    if (info_args.current->count != 0) {
        if (strcmp(info_args.current->sval[0], "mA") == 0)
            ESP_LOGI("CLI", "Displaying current, [mA].");
        else if (strcmp(info_args.current->sval[0], "A") == 0)
            ESP_LOGI("CLI", "Displaying current, [A].");
    }
    return 0;
}
static void register_info (void) {
    info_args.temp      = arg_str0("t", "temp",     "<C|F>",    "Displays the chip temperature.");
    info_args.voltage   = arg_str0("v", "voltage",  "<V|mV>",   "Displays the voltage.");
    info_args.current   = arg_str0("c", "current",  "<A|mA>",   "Displays the current.");
    //info_args.chip_temp = arg_lit0("T", "temperature",          "Prints the chip temperature.");
    info_args.end       = arg_end(2);

    const esp_console_cmd_t info_cmd = {
        .command    = "info",
        .help       = "Prints system information",
        .hint       = NULL,
        .func       = &exec_info_cmd,
        .argtable   = &info_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&info_cmd));
}


void register_commands (void) {
    register_info();
}

#endif