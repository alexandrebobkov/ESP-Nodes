/*
    ESPNOW RC Controller Module
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

void onDataSent (uint8_t *mac_addr, esp_now_send_status_t status);
void sensors_data_prepare(espnow_data_packet_t *send_packet);
static void rc_send_data_task (void *arg);
void sendData (void);
void sensors_data_prepare(espnow_data_packet_t *send_packet);

#endif