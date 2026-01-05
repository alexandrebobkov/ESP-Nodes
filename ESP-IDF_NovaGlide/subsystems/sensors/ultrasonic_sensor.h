#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"

// I2C Configuration for HC-SR04 I2C module
#define ULTRASONIC_I2C_ADDR         0x57    // Common I2C address for HC-SR04 I2C modules
#define ULTRASONIC_I2C_SPEED_HZ     100000  // 100kHz

// Forward declaration
typedef struct ultrasonic_system_t ultrasonic_system_t;

// Struct definition
struct ultrasonic_system_t {
    float distance_cm;
    i2c_master_dev_handle_t dev;  // I2C device handle

    void (*update)(ultrasonic_system_t *self, TickType_t now);
};

void ultrasonic_system_init(ultrasonic_system_t *sys);
void ultrasonic_system_init(ultrasonic_system_t *ultra);
void ultrasonic_probe_registers(ultrasonic_system_t *ultra);


#endif
