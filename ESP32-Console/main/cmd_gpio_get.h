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
    return 0;
}

#endif