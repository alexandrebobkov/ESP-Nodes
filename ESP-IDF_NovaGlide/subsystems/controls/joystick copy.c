#include "joystick.h"

static float clampf(float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}

void joystick_mix(int X_raw, int Y_raw, int *pwm_left, int *pwm_right) {
    // 1. Normalize joystick to [-1 .. +1]
    //float x = (float)(X_raw - 1020) / 1020.0f;
    //float y = (float)(Y_raw - 1020) / 1020.0f;

    float x = (float)(X_raw - 63635456) / 63635456.0f;
    float y = (float)(Y_raw - 66912256) / 66912256.0f;


    // 2. Steering gain for smooth arcs
    const float k = 0.5f;

    // 3. Raw differential mix
    float L0 = y + k * x;
    float R0 = y - k * x;

    // 4. Normalize pair so neither exceeds magnitude 1
    float m = fmaxf(1.0f, fmaxf(fabsf(L0), fabsf(R0)));
    float L = L0 / m;
    float R = R0 / m;

    // 5. Scale to signed PWM range [-8191 .. +8190]
    float L_scaled = L * 8190.0f;
    float R_scaled = R * 8190.0f;

    // 6. Clamp and output as integers
    *pwm_left = (int)clampf(L_scaled, -8191.0f, 8190.0f);
    *pwm_right = (int)clampf(R_scaled, -8191.0f, 8190.0f);
}
