#pragma once
#include "freertos/FreeRTOS.h"
#include "esp_now.h"

typedef struct espnow_system_t espnow_system_t;

struct espnow_system_t {
    // Last received data (you can expand this later)
    uint8_t last_data[32];
    int last_len;

    // API
    void (*send)(espnow_system_t *self, const uint8_t *data, int len);
    void (*update)(espnow_system_t *self, TickType_t now);
};

void espnow_system_init(espnow_system_t *sys);
