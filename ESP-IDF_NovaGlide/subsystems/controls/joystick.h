#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef struct joystick_hal_t joystick_hal_t;

struct joystick_hal_t {
    // Raw values from ESP-NOW or ADC
    int32_t raw_x;
    int32_t raw_y;

    // Learned calibration
    float center_x;
    float center_y;
    float range_x;
    float range_y;

    // Filtered normalized output [-1..+1]
    float norm_x;
    float norm_y;

    // Deadband threshold
    float deadband;

    // Calibration state
    int samples_collected;
    int samples_needed;

    // Update function
    void (*update)(joystick_hal_t *self, int32_t x_raw, int32_t y_raw);
};

// Public API
void joystick_hal_init(joystick_hal_t *js);
void joystick_hal_update(joystick_hal_t *js, int32_t x_raw, int32_t y_raw);
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right);


#endif
