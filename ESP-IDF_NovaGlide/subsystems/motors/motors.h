#pragma once
#include "freertos/FreeRTOS.h"

typedef struct motor_system_t motor_system_t;

struct motor_system_t {
    int pwm_left;
    int pwm_right;

    void (*set)(motor_system_t *self, int left, int right);
    void (*stop)(motor_system_t *self);
    void (*update)(motor_system_t *self, TickType_t now);
};

void motor_system_init(motor_system_t *sys);
