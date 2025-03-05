#ifndef CMD_SCAN_WIFI_H
#define CMD_SCAN_WIFI_H

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"

static struct {
    struct arg_int *ap_number;
//    struct arg_int *dump;
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
        // Scan Wi-Fi
        printf("\nScanning is in progress");
    }
    return s;
}

static void register_scan_wifi_cmd (void) {

    scan_wifi_args.ap_number      = arg_int0("n", "num", "<ap number>", "Specifies number of APs to scan.");
//    gpio_get_args.dump      = arg_int0("d", "dump", "<pin>", "Outputs the GPIO dump.");
    scan_wifi_args.end       = arg_end(5);
    const esp_console_cmd_t scan_wifi_cmd = {
        .command    = "scan-wifi",
        .help       = "Scans Wi-Fi access points.",
        .hint       = NULL,
        .func       = &exec_scan_wifi_cmd,
        .argtable   = &scan_wifi_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&scan_wifi_cmd));
}

#endif