#ifndef MOTOR_CONTROLS_H
#define MOTOR_CONTROLSC_H

// Interpolate value (x) based on raw reading, min/max limits.
/*

    Joystick scale:     4096    2048        0
    PWM scale:          8191    4096        0

    PWM Output:         +8191       0   -8191
*/
static int interpolate_raw_val (int raw) {
    int x;

    x = raw/2048;

    return x;
}
/*static int interpolate_raw_val (int min, int max, int raw) {
    int x;

    x = raw - 8191;

    return x;
}*/

#endif