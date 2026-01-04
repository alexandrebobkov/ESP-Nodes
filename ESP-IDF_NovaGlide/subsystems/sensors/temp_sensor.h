#pragma once
#include "freertos/FreeRTOS.h"

typedef struct temp_sensor_system_t temp_sensor_system_t;

struct temp_sensor_system_t {
    float last_celsius;
    void (*update)(temp_sensor_system_t *self, TickType_t now);
};

void temp_sensor_system_init(temp_sensor_system_t *sys);
