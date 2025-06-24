#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"

#include "config.h"


//extern uint8_t broadcast_mac[ESP_NOW_ETH_ALEN];     // Broadcast MAC address
//extern uint8_t receiver_mac[ESP_NOW_ETH_ALEN];      // MAC address of Robot
//extern uint8_t transmitter_mac[ESP_NOW_ETH_ALEN];   // MAC address of Remote Control

void wifi_init();
void transmission_init();

esp_err_t joystick_adc_init(void);
void joystick_show_raw_xy();
void get_joystick_xy(int *x_axis, int *y_axis);
void sendData (void);
void deletePeer (void);
void joystick_task(void *arg);
void statusDataSend(const uint8_t *mac_addr, esp_now_send_status_t status);

void rc_send_data_task();



#endif