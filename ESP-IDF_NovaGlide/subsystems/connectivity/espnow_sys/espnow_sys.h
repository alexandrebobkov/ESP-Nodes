#ifndef ESPNOW_SYS_H
#define ESPNOW_SYS_H

#include "freertos/FreeRTOS.h"
#include "esp_now.h"
#include "esp_wifi.h"

// Match your original structure EXACTLY
typedef struct {
    int x_axis;
    int y_axis;
    int motor1_rpm_pcm;
    int motor2_rpm_pcm;
    int motor3_rpm_pcm;
    int motor4_rpm_pcm;
} sensors_data_t;

// Forward declaration
typedef struct espnow_system_t espnow_system_t;

// Struct definition
struct espnow_system_t {
    sensors_data_t last_data;
    int last_len;

    void (*send)(espnow_system_t *self, const uint8_t *data, int len);
    void (*update)(espnow_system_t *self, TickType_t now);
};

void espnow_system_init(espnow_system_t *sys);

#endif
