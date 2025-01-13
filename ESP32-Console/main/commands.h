#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"

static int exec_info_cmd(int argc, char **argv);
static void register_info(void);

static int exec_set_gpio();
static void register_set_gpio(void);

struct arg_lit *temp, *help;
struct arg_end *end;
static struct {
    struct arg_str *detail;
    //struct arg_int *chip_temp;
    //struct arg_int *voltage;
    struct arg_end *end;
} info_args;
static int exec_info_cmd (int argc, char **argv) {

    void *argtable[] = {
        help = arg_litn(NULL, "help", 0, 1, "display this help"),
        temp = arg_litn("t", "temp", 0, 1, "chip temperature"),
        end = arg_end(20),
    };
    char progname[] = "info";
    //int nerrors;
    //nerrors = arg_parse(argc, argv, argtable);

    ESP_LOGW("CLI", "This is the Information Command.");

    int nerrors = arg_parse(argc, argv, (void**) &info_args);
    //int nerrors = arg_parse(argc, argv, argtable);
    if (nerrors != 0) {
        arg_print_errors(stderr, info_args.end, argv[0]);
        return 1;
    }

    if (info_args.detail->count != 0) {
        ESP_LOGW("CLI", "info(): Chip Temperature %d", 25);
    }
    return 0;
}
static void register_info (void) {
    info_args.detail = arg_str0(NULL, NULL, "<temp>", "Diplays the chip temperature.");
    //info_args.detail = arg_str1(NULL, "gpio", "<num>", "Diplays the GPIO level.");
    info_args.end = arg_end(2);

    const esp_console_cmd_t info_cmd = {
        .command = "info",
        .help = "Prints system information",
        .hint = NULL,
        .func = &exec_info_cmd,
        .argtable = &info_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&info_cmd));
}

void register_commands (void) {
    register_info();
}

#endif