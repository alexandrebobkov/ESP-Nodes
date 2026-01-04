#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "driver/temperature_sensor.h"

// Forward declaration
typedef struct temp_sensor_system_t temp_sensor_system_t;

// Struct definition
struct temp_sensor_system_t {
    float temperature;
    temperature_sensor_handle_t handle;

    void (*update)(temp_sensor_system_t *self, TickType_t now);
};

void temp_sensor_system_init(temp_sensor_system_t *sys);

#endif
