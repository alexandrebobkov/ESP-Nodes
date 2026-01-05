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

    // 1. Fixed center + range
    float dx = (float)x_raw - JS_CENTER_X;
    float dy = (float)y_raw - JS_CENTER_Y;

    float nx = dx / JS_RANGE_X;
    float ny = dy / JS_RANGE_Y;

    // 2. Clamp to [-1..+1]
    if (nx > 1.0f) nx = 1.0f;
    if (nx < -1.0f) nx = -1.0f;
    if (ny > 1.0f) ny = 1.0f;
    if (ny < -1.0f) ny = -1.0f;

    // 3. Strong deadband, especially on X
    const float deadband_x = 0.15f;
    const float deadband_y = 0.08f;

    js->norm_x = (fabsf(nx) < deadband_x) ? 0.0f : nx;
    js->norm_y = (fabsf(ny) < deadband_y) ? 0.0f : ny;
}

static float clampf(float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}

void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // 1. Nonlinear steering curve
    float x_shaped = x * x * x;   // soft near center, strong at edges
    // float x_shaped = x * fabsf(x);   // quadratic expo, stronger than cubic
    //float x_shaped = x * x * x * 1.2f; // stronger expo

    // 2. Steering gain
    const float k = 0.9f;   // Stronger steering; preferred 0.9

    // 3. Differential mix
    float L0 = y + k * x_shaped;
    float R0 = y - k * x_shaped;

    // 4. Limit left/right difference to 75%
    float diff = fabsf(L0 - R0);
        float max_diff = 1.8f;   // 75% of full 2.0 span // allow full spin but keep arcs smooth; had 1.2

    if (diff > max_diff) {
        float scale = max_diff / diff;
        L0 *= scale;
        R0 *= scale;
    }

    // 5. Clamp to [-1, 1] WITHOUT normalization
    if (L0 > 1.0f) L0 = 1.0f;
    if (L0 < -1.0f) L0 = -1.0f;
    if (R0 > 1.0f) R0 = 1.0f;
    if (R0 < -1.0f) R0 = -1.0f;

    // 6. Scale to PWM
    *pwm_left  = (int)(L0 * 8190.0f);
    *pwm_right = (int)(R0 * 8190.0f);
}
