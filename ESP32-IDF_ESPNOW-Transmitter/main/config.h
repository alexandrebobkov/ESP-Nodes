#ifndef CONFIG_H
#define CONFIG_H

/*
    Configuration variables
*/
//#define CONFIG_ESPNOW_CHANNEL           (1)                     // ESPNOW channel, default is 1
//#define CONFIG_ESPNOW_WIFI_MODE         (WIFI_MODE_STA)         // WiFi mode
//#define CONFIG_ESPNOW_WIFI_IF           (WIFI_IF_STA)           // WiFi interface
//#define CONFIG_ESPNOW_ENABLE_LONG_RANGE (0)                   // Enable long range mode
//#define ESPNOW_MAXDELAY 512

/*  ============================
    Joystick
    ============================ 
*/
#define PROJ_X                      (1)                     // ADC1_CH1; 0 GPIO joystick, x-axis
#define PROJ_Y                      (0)                     // ADC1_CH0; 1 GPIO joystick, y-axis
#define NAV_BTN                     (8)                     // 8 GPIO joystick button

const char *TAG = "ESP-NOW_Transmitter"; 

/*  ============================
            ESP NOW
    ============================

    ESP32-C3 Luatos ESP32C3 board MAC:      54:32:04:46:71:80
    ESP32-C3 SuperMini MAC:                 34:b7:da:f9:33:8d
    ESP32-C3 Breadboard MAC:                e4:b0:63:17:9e:45
*/
uint8_t broadcast_mac[ESP_NOW_ETH_ALEN]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};     // Broadcast MAC address
uint8_t receiver_mac[ESP_NOW_ETH_ALEN]       = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot
uint8_t transmitter_mac[ESP_NOW_ETH_ALEN]    = {0x34, 0xB7, 0xDA, 0xF9, 0x33, 0x8D};     // MAC address of Remote Control

#endif