#ifndef CMD_GPIO_SET_H
#define CMD_GPIO_SET_H

#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"

// Set GPIOs states
static int exec_gpio_cmd(int argc, char **argv);
static void register_gpio(void);
gpio_config_t pin_config;

// Struct holding the ommand arguments
static struct {
    struct arg_int *gpio;   // Stores GPIO number
    struct arg_str *mode;   // Stores Input or Output mode
    struct arg_int *level;  // Stores HIGH or LOW level
    struct arg_int *pwm;    // Stores cycle duty value (PWM)
    struct arg_end *end;    // Stores parsing errors
} gpio_set_args;

// Function being called when command 'gpio-set' is entered.
static int exec_gpio_set_cmd (int argc, char **argv) {

    int nerrors = arg_parse(argc, argv, (void**) &gpio_set_args);
    // Confirm there were no parsing errors
    if (nerrors != 0) {
        arg_print_errors(stderr, gpio_set_args.end, argv[0]);
        return 1;
    }
    else {
        /*
            Set the levelof specified GPIO HIGH or LOW
        */
        // Check that the minimal required arguments were passed, i.e. pin #, mode, and logic level.
        if (gpio_set_args.gpio->count > 0 && gpio_set_args.level->count > 0 && gpio_set_args.mode->count > 0) {
            // Display brief summary of passed arguments.
            ESP_LOGI("GPIO", "pin: %i, mode: %s, level: %i",
                gpio_set_args.gpio->ival[0],
                gpio_set_args.mode->sval[0],
                gpio_set_args.level->ival[0]);
            // Set the bit mask for the pin number that was passed
            pin_config.pin_bit_mask = 1ULL << gpio_set_args.gpio->ival[0];
            // Since we are setting the pin level, set the pin mode to OUTPUT
            pin_config.mode = GPIO_MODE_OUTPUT;
            // Submit pin configuration
            gpio_config(&pin_config);
            // Change the pin level
            gpio_set_level(gpio_set_args.gpio->ival[0], gpio_set_args.level->ival[0]);
        }
    }
    return 0;
}
// Function registering the 'gpio-set' command with its arguments.
static void register_gpio_set_cmd (void) {

    gpio_set_args.gpio  = arg_int0("p", "gpio",     "<pin>",    "Specifies GPIO to be used");
    gpio_set_args.mode  = arg_str0("m", "mode",     "<in|out>", "Sets the mode of GPIO.");
    gpio_set_args.level = arg_int0("l", "level",    "<1|0>",    "Sets the logical level of GPIO.");
    gpio_set_args.pwm   = arg_int0("f", "pwm",      "<num>",    "Set PWM for the given GPIO.");
    gpio_set_args.end   = arg_end(2);
    const esp_console_cmd_t gpio_set_cmd = {
        .command    = "gpio-set",
        .help       = "Sets GPIOs logic levels",
        .hint       = NULL,
        .func       = &exec_gpio_set_cmd,
        .argtable   = &gpio_set_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&gpio_set_cmd));
}

#endif