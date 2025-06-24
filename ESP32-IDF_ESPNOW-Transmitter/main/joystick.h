#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"

#include "config.h"

void wifi_init();
esp_err_t joystick_adc_init(void);
void transmission_init();

static void rc_send_data_task();
static void sendData (void);
static void get_joystick_xy(int *x_axis, int *y_axis);
static void deletePeer (void);
static void statusDataSend(const uint8_t *mac_addr, esp_now_send_status_t status);

//void joystick_show_raw_xy();

#endif