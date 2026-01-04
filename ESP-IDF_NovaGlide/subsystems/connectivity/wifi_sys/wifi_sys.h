#ifndef WIFI_SYS_H
#define WIFI_SYS_H

#include "esp_wifi.h"
#include "esp_event.h"

#define WIFI_SSID "IoT_bots"
#define WIFI_PASSWORD "208208208"
#define ESPNOW_CHANNEL 1

void wifi_system_init(void);

#endif
