#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"

#include "config.h"

void wifi_init();
void transmission_init();
void setData(void);

static void rc_send_data_task();
static void sendData (void);
static void deletePeer (void);
static void statusDataSend(const uint8_t *mac_addr, esp_now_send_status_t status);

#endif