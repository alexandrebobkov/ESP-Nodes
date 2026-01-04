#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "driver/temperature_sensor.h"

typedef struct {
    float temperature;
    temperature_sensor_handle_t handle;

    void (*update)(struct temp_sensor_system_t *self, TickType_t now);
} temp_sensor_system_t;

void temp_sensor_system_init(temp_sensor_system_t *sys);

#endif
