#pragma once
#include "freertos/FreeRTOS.h"

typedef struct ina219_system_t ina219_system_t;

struct ina219_system_t {
    float voltage;
    float current;
    float power;

    void (*update)(ina219_system_t *self, TickType_t now);
};

void ina219_system_init(ina219_system_t *sys);
