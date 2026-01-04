#pragma once
#include "freertos/FreeRTOS.h"

typedef struct ultrasonic_system_t ultrasonic_system_t;

struct ultrasonic_system_t {
    float distance_cm;
    void (*update)(ultrasonic_system_t *self, TickType_t now);
};

void ultrasonic_system_init(ultrasonic_system_t *sys);
