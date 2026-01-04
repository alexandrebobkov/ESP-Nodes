#include "ina219_sensor.h"
#include "esp_log.h"

static const char *TAG = "INA219";

static void ina_update_impl(ina219_system_t *self, TickType_t now)
{
    (void)now;
    self->voltage = 12.0f;
    self->current = 0.5f;
    self->power = self->voltage * self->current;
}

void ina219_system_init(ina219_system_t *sys)
{
    sys->voltage = 0;
    sys->current = 0;
    sys->power = 0;
    sys->update = ina_update_impl;
}
