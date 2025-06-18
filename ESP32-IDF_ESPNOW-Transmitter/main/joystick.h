#ifndef JOYSTICK_H
#define JOYSTICK_H

extern uint8_t broadcast_mac[ESP_NOW_ETH_ALEN];//      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};     // Broadcast MAC address
extern uint8_t receiver_mac[ESP_NOW_ETH_ALEN];//       = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot
extern uint8_t transmitter_mac[ESP_NOW_ETH_ALEN];//    = {0x34, 0xB7, 0xDA, 0xF9, 0x33, 0x8D};     // MAC address of Remote Control

esp_err_t joystick_adc_init(void);
void joystick_show_raw_xy();
void get_joystick_xy(int *x_axis, int *y_axis);
void sendData (void);
void deletePeer (void);
void joystick_task(void *arg);
void statusDataSend(const uint8_t *mac_addr, esp_now_send_status_t status);
void wifi_init();
void rc_send_data_task();


#endif