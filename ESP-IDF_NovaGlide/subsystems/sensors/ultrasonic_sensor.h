#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

// I2C address for ultrasonic sensor (common addresses: 0x57, 0x70, or check your datasheet)
#define ULTRASONIC_I2C_ADDR 0x57

typedef struct ultrasonic_system_s {
    float distance_cm;           // Measured distance in centimeters
    bool measurement_valid;      // True if last measurement was successful
    void (*update)(struct ultrasonic_system_s *self, TickType_t now);
} ultrasonic_system_t;

/**
 * Initialize the I2C ultrasonic sensor system
 * @param sys Pointer to ultrasonic system structure
 */
void ultrasonic_system_init(ultrasonic_system_t *sys);

#endif // ULTRASONIC_H
