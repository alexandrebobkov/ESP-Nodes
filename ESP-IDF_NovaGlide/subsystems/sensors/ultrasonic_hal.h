#ifndef ULTRASONIC_HAL_H
#define ULTRASONIC_HAL_H

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"

typedef struct ultrasonic_hal_t ultrasonic_hal_t;

struct ultrasonic_hal_t {
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;

    rmt_channel_handle_t rmt_tx;
    rmt_channel_handle_t rmt_rx;
    rmt_encoder_handle_t encoder;

    float distance_cm;

    void (*update)(ultrasonic_hal_t *self, TickType_t now);
};

void ultrasonic_hal_init(ultrasonic_hal_t *ultra,
                         gpio_num_t trig_pin,
                         gpio_num_t echo_pin);

#endif
