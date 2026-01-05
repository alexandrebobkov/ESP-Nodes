#ifndef ULTRASONIC_HAL_H
#define ULTRASONIC_HAL_H

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"

// Unified subsystem type (matches your architecture)
typedef struct ultrasonic_system_t ultrasonic_system_t;

struct ultrasonic_system_t {
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;

    rmt_channel_handle_t rmt_tx;
    rmt_channel_handle_t rmt_rx;

    float distance_cm;

    void (*update)(ultrasonic_system_t *self, TickType_t now);
};

void ultrasonic_system_init(ultrasonic_system_t *ultra,
                            gpio_num_t trig_pin,
                            gpio_num_t echo_pin);

#endif
