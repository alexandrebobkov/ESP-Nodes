#pragma once
#include "freertos/FreeRTOS.h"

// Forward declarations of subsystem structs
typedef struct motor_system_t       motor_system_t;
typedef struct adc_system_t         adc_system_t;
typedef struct mqtt_system_t        mqtt_system_t;
typedef struct temp_sensor_system_t temp_sensor_system_t;
typedef struct ina219_system_t      ina219_system_t;
typedef struct ultrasonic_system_t  ultrasonic_system_t;

typedef struct scheduler_t {
    motor_system_t       *motors;
    adc_system_t         *adc;
    mqtt_system_t        *mqtt;
    temp_sensor_system_t *temp;
    ina219_system_t      *ina;
    ultrasonic_system_t  *ultra;
} scheduler_t;

void scheduler_init(scheduler_t *sched);
void scheduler_start(scheduler_t *sched);
