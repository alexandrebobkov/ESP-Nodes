#pragma once
#include "freertos/FreeRTOS.h"

typedef struct adc_system_t adc_system_t;

struct adc_system_t {
    int x_raw;
    int y_raw;

    void (*read)(adc_system_t *self);
    void (*update)(adc_system_t *self, TickType_t now);
};

void adc_system_init(adc_system_t *sys);
