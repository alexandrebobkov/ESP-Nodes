#include "ultrasonic_sensor.h"
#include "esp_log.h"

static const char *TAG = "ULTRASONIC";

static void ultra_update_impl(ultrasonic_system_t *self, TickType_t now)
{
    (void)now;
    self->distance_cm = 42.0f;
}

void ultrasonic_system_init(ultrasonic_system_t *sys)
{
    sys->distance_cm = 0.0f;
    sys->update = ultra_update_impl;
}
