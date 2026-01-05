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
    // float x_shaped = x * fabsf(x);   // quadratic expo, stronger than cubic
    //float x_shaped = x * x * x * 1.2f; // stronger expo
    float x_shaped = x * x * x;
    // ^ Cubic expo curve:
    //   - Near the joystick center (x ≈ 0), x³ becomes extremely small.
    //     Example: 0.2³ = 0.008 → almost no steering.
    //   - As x approaches ±1, the curve ramps up sharply.
    //     Example: 0.8³ = 0.512 → strong steering.
    //   This gives smooth, gentle arcs for small stick movements,
    //   while still allowing full turning authority at full stick deflection.
    //
    //   The two commented alternatives:
    //     x * fabsf(x)      → quadratic expo, softer than linear but stronger than cubic.
    //     x³ * 1.2f         → cubic expo with extra gain, more aggressive turning.
    //
    //   You chose pure cubic, which gives the smoothest center feel.

    // 2. Steering gain
    const float k = 0.9f;
    // ^ Steering gain multiplier:
    //   - Controls how strongly the shaped X value influences turning.
    //   - Lower values (0.5–0.7) → softer, wider arcs.
    //   - Higher values (1.0–1.3) → more aggressive turning and faster spins.
    //   - Your chosen 0.9 gives a balanced feel: smooth arcs but still strong spin capability.

    // 3. Differential mix
    float L0 = y + k * x_shaped;
    float R0 = y - k * x_shaped;
    // ^ Classic differential drive mixing:
    //   - y controls forward/backward speed.
    //   - x_shaped controls turning.
    //   - Adding to left / subtracting from right creates a turn.
    //   - When y = 0 and x = ±1, this produces a pure spin-in-place.
    //   - When both x and y are present, you get an arc.

    // 4. Limit left/right difference to 75%
    float diff = fabsf(L0 - R0);
    float max_diff = 1.7f;
    // ^ Differential limiter:
    //   - The maximum possible difference between L0 and R0 is 2.0
    //       (L0 = +1, R0 = -1 → diff = 2)
    //   - max_diff = 1.7 caps the turning strength to ~85% of maximum.
    //   - This prevents the robot from snapping too aggressively into a turn.
    //   - It also keeps arcs smooth and prevents sudden PWM drops.
    //   - Your previous value (1.2) limited turning to ~60%, which felt too weak.
    //   - 1.7 allows near-full spin while still smoothing out harsh transitions.

    if (diff > max_diff) {
        float scale = max_diff / diff;
        L0 *= scale;
        R0 *= scale;
    }
    // ^ If the left/right difference exceeds the allowed limit,
    //   both sides are scaled down proportionally.
    //   This preserves the *shape* of the turn while reducing its intensity.
    //   It prevents sudden jumps in PWM and keeps steering predictable.


    // 5. Clamp to [-1, 1] WITHOUT normalization
    if (L0 > 1.0f) L0 = 1.0f;
    if (L0 < -1.0f) L0 = -1.0f;
    if (R0 > 1.0f) R0 = 1.0f;
    if (R0 < -1.0f) R0 = -1.0f;

    // 6. Scale to PWM
    *pwm_left  = (int)(L0 * 8190.0f);
    *pwm_right = (int)(R0 * 8190.0f);
}
