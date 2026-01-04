#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "ultrasonic.h"

#define ULTRASONIC_TRIGGER_GPIO 4
#define ULTRASONIC_ECHO_GPIO 5
#define PING_TIMEOUT 6000
#define ROUNDTRIP_CM 58.0f

typedef struct {
    float distance_cm;
    ultrasonic_sensor_t sensor;

    void (*update)(struct ultrasonic_system_t *self, TickType_t now);
} ultrasonic_system_t;

void ultrasonic_system_init(ultrasonic_system_t *sys);

#endif
