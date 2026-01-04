#pragma once
#include "freertos/FreeRTOS.h"

typedef struct mqtt_system_t mqtt_system_t;

struct mqtt_system_t {
    float temp;
    int pwm_1;
    int pwm_2;

    void (*publish_temp)(mqtt_system_t *self, float t);
    void (*publish_pwm)(mqtt_system_t *self, int p1, int p2);
    void (*update)(mqtt_system_t *self, TickType_t now);
};

void mqtt_system_init(mqtt_system_t *sys);
