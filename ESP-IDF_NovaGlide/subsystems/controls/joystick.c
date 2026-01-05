#include "joystick.h"

static float apply_deadband(float v, float d)
{
    return (fabsf(v) < d) ? 0.0f : v;
}

void joystick_hal_init(joystick_hal_t *js)
{
    js->raw_x = 0;
    js->raw_y = 0;

    js->center_x = 0;
    js->center_y = 0;

    js->range_x = 1;
    js->range_y = 1;

    js->norm_x = 0;
    js->norm_y = 0;

    js->deadband = 0.05f;        // 5% deadband
    js->samples_collected = 0;
    js->samples_needed = 200;    // learn center over first 200 samples

    js->update = joystick_hal_update;
}

void joystick_hal_update(joystick_hal_t *js, int32_t x_raw, int32_t y_raw)
{
    js->raw_x = x_raw;
    js->raw_y = y_raw;

    // --- 1. Learn center for first N samples ---
    if (js->samples_collected < js->samples_needed) {
        js->center_x += x_raw;
        js->center_y += y_raw;
        js->samples_collected++;

        if (js->samples_collected == js->samples_needed) {
            js->center_x /= js->samples_needed;
            js->center_y /= js->samples_needed;
        }

        js->norm_x = 0;
        js->norm_y = 0;
        return;
    }

    // --- 2. Compute deltas ---
    float dx = (float)x_raw - js->center_x;
    float dy = (float)y_raw - js->center_y;

    // --- 3. Learn range dynamically ---
    js->range_x = fmaxf(js->range_x, fabsf(dx));
    js->range_y = fmaxf(js->range_y, fabsf(dy));

    // --- 4. Normalize to [-1..+1] ---
    float nx = dx / js->range_x;
    float ny = dy / js->range_y;

    // --- 5. Apply deadband ---
    js->norm_x = apply_deadband(nx, js->deadband);
    js->norm_y = apply_deadband(ny, js->deadband);
}

static float clampf(float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}

void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // Steering gain
    const float k = 0.5f;

    // Differential mix
    float L0 = y + k * x;
    float R0 = y - k * x;

    // Normalize pair
    float m = fmaxf(1.0f, fmaxf(fabsf(L0), fabsf(R0)));
    float L = L0 / m;
    float R = R0 / m;

    // Scale to PWM range
    float L_scaled = L * 8190.0f;
    float R_scaled = R * 8190.0f;

    // Clamp and output
    *pwm_left  = (int)clampf(L_scaled, -8191.0f, 8190.0f);
    *pwm_right = (int)clampf(R_scaled, -8191.0f, 8190.0f);
}
