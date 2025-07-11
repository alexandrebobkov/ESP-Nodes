#include "config.h"

uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t receiver_mac[ESP_NOW_ETH_ALEN]  = {0x9c, 0x9e, 0x6e, 0x14, 0xb5, 0x54};
// ESP32-C3 Breadboard Robot Receiver
//uint8_t receiver_mac[ESP_NOW_ETH_ALEN]  = {0xe4, 0xb0, 0x63, 0x17, 0x9e, 0x45};
uint8_t receiver_mac[ESP_NOW_ETH_ALEN]  = {0xe4, 0xb0, 0x63, 0x17, 0x9e, 0x44};

//uint8_t transmitter_mac[ESP_NOW_ETH_ALEN] = {0x34, 0xB7, 0xDA, 0xF9, 0x33, 0x8D};
//uint8_t transmitter_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t transmitter_mac[ESP_NOW_ETH_ALEN] = {0x9C, 0x9E, 0x6E, 0x14, 0xB5, 0x54};

const char *TAG = "ESP-NOW_Receiver";