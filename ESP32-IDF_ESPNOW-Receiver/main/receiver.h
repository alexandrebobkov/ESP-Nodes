#ifndef RECEIVER_H
#define RECEIVER_H

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"

void wifi_init(void);
void transmission_init(void);
void onDataReceived (const uint8_t *mac_addr, const uint8_t *data, uint8_t data_len);

#endif