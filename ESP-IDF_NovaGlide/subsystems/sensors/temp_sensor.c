#include "temp_sensor.h"

void temp_sensor_system_init (temp_sensor_system_t *sys) {
	sys->last_celsius = 0.0f;
	sys->update = NULL;
}
