#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

void chip_sensor_init ();
void system_led_init ();
void get_chip_temperature(float *temperature_value);
static void system_led_task (void *arg);
static void temp_sensor_task (void *arg);

#endif