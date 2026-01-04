#include "motors.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTORS";

static void motor_set_impl(motor_system_t *self, int left, int right)
{
    self->pwm_left  = left;
    self->pwm_right = right;
}

static void motor_stop_impl(motor_system_t *self)
{
    motor_set_impl(self, 0, 0);
}

static void motor_update_impl(motor_system_t *self, TickType_t now)
{
    (void)now;
}

void motor_system_init(motor_system_t *sys)
{
    sys->pwm_left = 0;
    sys->pwm_right = 0;

    sys->set = motor_set_impl;
    sys->stop = motor_stop_impl;
    sys->update = motor_update_impl;
}
