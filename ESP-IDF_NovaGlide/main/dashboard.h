// dashboard.h
#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "freertos/FreeRTOS.h"
#include "motors.h"
#include "espnow_sys.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_hal.h"

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    ultrasonic_system_t *ultrasonic;
} dashboard_context_t;

void dashboard_task_start(dashboard_context_t *ctx);

#endif
