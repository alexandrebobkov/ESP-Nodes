#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include "esp_now.h"

/*
    Configuration variables
*/

#define SYS_LED_GPIO  (10)  // 10 GPIO of on-board LED

/*  ============================
    Joystick
    ============================ 
*/
#define PROJ_X                      (1)                     // ADC1_CH1; 0 GPIO joystick, x-axis
#define PROJ_Y                      (0)                     // ADC1_CH0; 1 GPIO joystick, y-axis
#define NAV_BTN                     (8)                     // 8 GPIO joystick button

/*  ============================
            ESP NOW
    ============================

    ESP32-C3 Luatos ESP32C3 board MAC:      54:32:04:46:71:80
    ESP32-C3 SuperMini MAC:                 34:b7:da:f9:33:8d
    ESP32-C3 Breadboard #1 MAC:             e4:b0:63:17:9e:45
    ESP32-C3 Breadboard #2 MAC:             9c:9e:6e:14:b5:54 (rapid blink)
    ESP32-C3 zenBoard MAC:                  dc:da:0c:8e:b3:70
*/

extern uint8_t broadcast_mac[ESP_NOW_ETH_ALEN];
extern uint8_t receiver_mac[ESP_NOW_ETH_ALEN];
extern uint8_t receiver_2_mac[ESP_NOW_ETH_ALEN];
extern uint8_t transmitter_mac[ESP_NOW_ETH_ALEN];
extern const char *TAG;
#endif