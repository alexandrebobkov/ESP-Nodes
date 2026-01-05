#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"
#include "adc.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic.h"
#include "mqtt_sys.h"
#include "espnow_sys.h"
#include "ui.h"

typedef struct {
    motor_system_t *motors;
    adc_system_t *adc;
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    ultrasonic_sensor_t *ultra;
    mqtt_system_t *mqtt;
    espnow_system_t *espnow;
    ui_system_t *ui;
} scheduler_t;

void scheduler_init(scheduler_t *sched);
void scheduler_start(scheduler_t *sched);

#endif
