/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_POWER  true
#define REPORTING_PERIOD 300 //60

extern esp_rmaker_device_t *switch_device;
// Define RainMaker device: switch
extern esp_rmaker_device_t *switch_device;
// Define RainMaker device: temperature gauge
extern esp_rmaker_device_t *light_gauge_device;
extern esp_rmaker_device_t *temp_sensor_device;
extern esp_rmaker_device_t *chip_sensor_device; // internal temperature sensor

void app_driver_init(void);
void app_sensor_init(void);
int app_driver_set_state(bool state);
bool app_driver_get_state(void);

float app_get_current_temperature();
float app_get_internal_temperature();
