#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "ultrasonic.h"

#define ULTRASONIC_TRIGGER_GPIO 4
#define ULTRASONIC_ECHO_GPIO 5
#define PING_TIMEOUT 6000
#define ROUNDTRIP_CM 58.0f

#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_FREQ_HZ         100000
#define ULTRASONIC_I2C_ADDR 0x57   // HC-SR04 I2C mode


// Forward declaration
typedef struct ultrasonic_system_t ultrasonic_system_t;

// Struct definition
struct ultrasonic_system_t {
    float distance_cm;
    ultrasonic_sensor_t sensor;

    void (*update)(ultrasonic_system_t *self, TickType_t now);
};

void ultrasonic_system_init(ultrasonic_system_t *sys);

#endif
