#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

typedef struct {
    gpio_num_t trigger_gpio;  // GPIO for the trigger pin
    gpio_num_t echo_gpio;     // GPIO for the echo pin
} ultrasonic_sensor_t;

#endif