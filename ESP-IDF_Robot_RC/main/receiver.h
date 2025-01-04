/*
    ESPNOW RC Controller Module
*/

#ifndef RECEIVER_H
#define RECEIVER_H

void onDataReceived (uint8_t *mac_addr, uint8_t *data, uint8_t data_len);

#endif