/*
    ESPNOW RC Controller Module
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "common.h"

void onDataSent (uint8_t *mac_addr, esp_now_send_status_t status);
void sensors_data_prepare(espnow_data_packet_t *send_packet);
static void rc_send_data_task (void *arg);
static void rc_send_data_task2 (void *pvParameter);
void sendData (void);
void sensors_data_prepare(espnow_data_packet_t *send_packet);
void deletePeer (void);
static esp_err_t rc_espnow_init (void);

#endif