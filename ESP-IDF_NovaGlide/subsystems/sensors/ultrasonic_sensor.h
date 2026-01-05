#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#define ULTRASONIC_I2C_ADDR 0x57

typedef struct ultrasonic_system_t {
    float distance_cm;
    i2c_master_dev_handle_t dev;
    void (*update)(struct ultrasonic_system_t *self, TickType_t now);
} ultrasonic_system_t;

void ultrasonic_system_init(ultrasonic_system_t *sys);

#endif
