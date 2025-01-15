#ifndef CMD_GPIO_GET_H
#define CMD_GPIO_GET_H

#include <stdio.h>
#include <string.h>


#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"

static struct {
    struct arg_int *gpio;
    struct arg_end *end;
} gpio_get_args;

// Operate GPIOs states
static int exec_gpio_get_cmd(int argc, char **argv);
static void register_gpio_get_cmd(void);

static int exec_gpio_get_cmd(int argc, char **argv) {

    printf("Getting GPIO status ...\n");

    gpio_dump_io_configuration(stdout, 1ULL << 8);
    return 0;
}

static void register_gpio_get_cmd (void) {

    gpio_get_args.gpio      = arg_int0("p", "gpio", "<pin>", "Specifies GPIO to be used.");
    gpio_get_args.end       = arg_end(2);
    const esp_console_cmd_t gpio_get_cmd = {
        .command    = "gpio-get",
        .help       = "Gets GPIOs logic levels",
        .hint       = NULL,
        .func       = &exec_gpio_get_cmd,
        .argtable   = &gpio_get_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&gpio_get_cmd));
}

#endif