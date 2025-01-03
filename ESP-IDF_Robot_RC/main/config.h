/*
    Configuration variables
*/

#ifndef CONFIG_H
#define CONFIG_H

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
    ESP32-C3 Breadboard MAC:                e4:b0:63:17:9e:45
*/
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t robot_mac[ESP_NOW_ETH_ALEN]      = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot
static uint8_t rc_mac[ESP_NOW_ETH_ALEN]         = {0x34, 0xB7, 0xDA, 0xF9, 0x33, 0x8D};     // MAC address of Remote Control
static uint8_t receiver_mac[ESP_NOW_ETH_ALEN]   = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};

#endif