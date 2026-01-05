#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

typedef struct ultrasonic_system_s {
    float distance_cm;           // Measured distance in centimeters
    bool measurement_valid;      // True if last measurement was successful
    void (*update)(struct ultrasonic_system_s *self, TickType_t now);
} ultrasonic_system_t;

/**
 * Initialize the I2C ultrasonic sensor
 * Uses I2C address 0x57 with command 0x50 for cm measurements
 */
void ultrasonic_system_init(ultrasonic_system_t *sys);

#endif // ULTRASONIC_SENSOR_H
