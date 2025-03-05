#ifndef CMD_SCAN_WIFI_H
#define CMD_SCAN_WIFI_H

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"

static struct {
//    struct arg_int *gpio;
    struct arg_int *dump;
    struct arg_end *end;
} scan_wifi_args;

static int exec_scan_wifi_cmd(int argc, char **argv);
static void register_scan_wifi_cmd(void);

static int exec_scan_wifi_cmd(int argc, char **argv) {

    printf("Scanning Wi-Fi ...\n");

    int nerrors = arg_parse(argc, argv, (void**) &scan_wifi_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, scan_wifi_args.end, argv[0]);
        s = 1;
    }
    else {
        // Gets GPIO level
        if (gpio_get_args.gpio->count > 0) {
            pin = gpio_get_args.gpio->ival[0];
            printf("\nGPIO: %d \n", pin);
            //ESP_LOGI("gpio-get", "GPIO # %i", gpio_get_args.gpio->ival[0]);
            //gpio_dump_io_configuration(stdout, 1ULL << gpio_get_args.gpio->ival[0]);
            printf("GPIO #%d level is %d\n", gpio_get_args.gpio->ival[0], gpio_get_level(gpio_get_args.gpio->ival[0]));
            ESP_LOGW("gpio-get", "%d ", gpio_get_level(GPIO_NUM_8));
            //printf("GPIO level was changed.");
            s = 0;
        }
        // Dump GPIO configuration information.
        if (gpio_get_args.dump->count > 0) {
            printf("Printing GPIO dump ...\n");
            printf("GPDIO: %d \n", gpio_get_args.dump->ival[0]);
            gpio_dump_io_configuration(stdout, 1ULL << gpio_get_args.dump->ival[0]);
            s = 0;
        }
    }
    return s;
}

static void register_scan_wifi_cmd (void) {

    gpio_get_args.gpio      = arg_int0("p", "pin", "<pin>", "Specifies GPIO to be used.");
    gpio_get_args.dump      = arg_int0("d", "dump", "<pin>", "Outputs the GPIO dump.");
    gpio_get_args.end       = arg_end(5);
    const esp_console_cmd_t gpio_get_cmd = {
        .command    = "gpio-get",
        .help       = "Displays information about GPIOs",
        .hint       = NULL,
        .func       = &exec_gpio_get_cmd,
        .argtable   = &gpio_get_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&gpio_get_cmd));
}

#endif