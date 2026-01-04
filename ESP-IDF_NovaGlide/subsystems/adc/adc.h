#ifndef ADC_H
#define ADC_H

#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_oneshot.h"

// Forward declaration
typedef struct adc_system_t adc_system_t;

// Struct definition
struct adc_system_t {
    int last_reading;
    int x_raw;  // Joystick X-axis raw value
    int y_raw;  // Joystick Y-axis raw value

    void (*update)(adc_system_t *self, TickType_t now);
    void (*read)(adc_system_t *self, int *x, int *y);
};

void adc_system_init(adc_system_t *sys);

#endif
