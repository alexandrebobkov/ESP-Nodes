#ifndef CMD_SCAN_WIFI_H
#define CMD_SCAN_WIFI_H

static struct {
//    struct arg_int *gpio;
    struct arg_int *dump;
    struct arg_end *end;
} scan_wifi_args;

static int exec_scan_wifi_cmd(int argc, char **argv);
static void register_scan_wifi_cmd(void);

#endif