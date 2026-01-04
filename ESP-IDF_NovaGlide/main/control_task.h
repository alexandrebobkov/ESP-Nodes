#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "motors.h"
#include "espnow_sys.h"

void control_task_start(motor_system_t *motors, espnow_system_t *espnow);

#endif
