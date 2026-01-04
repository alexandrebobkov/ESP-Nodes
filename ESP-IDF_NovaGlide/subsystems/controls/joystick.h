#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <math.h>

// Joystick mixing algorithm
void joystick_mix(int X_raw, int Y_raw, int *pwm_left, int *pwm_right);

#endif
