#ifndef COMMANDS_H
#define COMMANDS_H

static int do_info_cmd (int argc, char **argv) {
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

#endif