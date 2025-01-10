#ifndef COMMANDS_H
#define COMMANDS_H

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"

static struct {
    struct arg_int *detail;
    struct arg_int *chip_temp;
    struct arg_int *voltage;
} info_args;
static int do_info_cmd (int argc, char **argv) {
    ESP_LOGW("CLI", "This is the Information Command.");

    if (info_args.chip_temp->count) {
        ESP_LOGW("CLI", "Information: Chip Temperature %d", 25);
    }
    return 0;
}
static void register_info (void) {
    const esp_console_cmd_t info_cmd = {
        .command = "info",
        .help = "System Information",
        .hint = NULL,
        .func = &do_info_cmd,
        .argtable = &info_args
    };
}

void register_commands (void) {
    register_info();
}

#endif