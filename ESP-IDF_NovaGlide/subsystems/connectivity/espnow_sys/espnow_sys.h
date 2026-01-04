#ifndef ESPNOW_SYS_H
#define ESPNOW_SYS_H

#include "freertos/FreeRTOS.h"
#include "esp_now.h"
#include "esp_wifi.h"

typedef struct {
    int x_axis;  // Joystick X (rc_x)
    int y_axis;  // Joystick Y (rc_y)
} espnow_joystick_data_t;

// Forward declaration
typedef struct espnow_system_t espnow_system_t;

// Struct definition
struct espnow_system_t {
    espnow_joystick_data_t last_data;
    int last_len;

    void (*send)(espnow_system_t *self, const uint8_t *data, int len);
    void (*update)(espnow_system_t *self, TickType_t now);
};

void espnow_system_init(espnow_system_t *sys);

#endif
