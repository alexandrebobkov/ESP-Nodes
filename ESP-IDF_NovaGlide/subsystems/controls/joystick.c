/**
 * @file joystick.c
 * @brief Advanced Joystick Input Processing and Differential Drive Mixing
 *
 * This module provides sophisticated joystick input handling with:
 * - Automatic calibration and center-point detection
 * - Asymmetric deadband zones for drift elimination
 * - Non-linear response curves for smooth control
 * - Differential drive mixing with arc limiting
 *
 * Control Philosophy:
 * The goal is to create intuitive, predictable robot control where:
 * - Small joystick movements produce gentle arcs (not jerky turns)
 * - Large movements allow aggressive maneuvers (spins, tight turns)
 * - The robot never does unexpected things due to joystick drift
 * - Control feels natural and responsive, not robotic
 *
 * Mathematical Approach:
 * 1. Raw ADC values → Normalized [-1, +1] coordinates
 * 2. Deadband filtering → Eliminates drift and centering issues
 * 3. Non-linear shaping → Creates smooth response curves
 * 4. Differential mixing → Converts X/Y to left/right motor commands
 * 5. Arc limiting → Prevents overly aggressive turning
 * 6. PWM scaling → Converts to motor controller values (0-8191)
 *
 * @author Alexander Bobkov
 * @date January 2026
 */

#include "joystick.h"

/**
 * @brief Apply deadband filter to eliminate drift
 *
 * Joystick hardware is never perfectly centered. Even at "rest", the ADC
 * readings fluctuate (±50 counts is typical). Without a deadband, this
 * causes constant micro-movements and motor whine.
 *
 * The deadband creates a "dead zone" around zero where small values are
 * treated as exactly zero. This eliminates drift while preserving
 * intentional control inputs.
 *
 * Mathematical behavior:
 * - If |v| < d: output = 0 (inside deadband)
 * - If |v| ≥ d: output = v (outside deadband, pass through unchanged)
 *
 * Trade-offs:
 * - Too small (d < 0.03): Drift not eliminated, motors buzz
 * - Too large (d > 0.15): Loss of fine control, "dead spots" feel bad
 * - Sweet spot: 0.05-0.10 (5-10% of full range)
 *
 * @param v Input value (typically -1.0 to +1.0)
 * @param d Deadband threshold (e.g., 0.05 = 5%)
 * @return Filtered value: 0 if within deadband, otherwise unchanged
 *
 * @note This is a simple rectangular deadband. More advanced alternatives:
 *       - Circular deadband (magnitude-based)
 *       - Gradient deadband (smooth transition, not hard cutoff)
 *       - Hysteresis deadband (prevents rapid on/off oscillation)
 */
static float apply_deadband(float v, float d)
{
    return (fabsf(v) < d) ? 0.0f : v;
}

/**
 * @brief Initialize joystick HAL (Hardware Abstraction Layer)
 *
 * Sets up the joystick system with:
 * - Zero initial values (safe state)
 * - Default calibration parameters
 * - Learning mode configuration
 *
 * Calibration Strategy:
 * The system can auto-learn the joystick center point by averaging the
 * first N samples (typically N=200, collected over 2-5 seconds). This
 * compensates for:
 * - Manufacturing tolerances (no two joysticks are identical)
 * - Mechanical wear (center drifts over time)
 * - Electrical noise (ADC reference variations)
 * - Temperature effects (resistance changes with temp)
 *
 * Current Implementation:
 * Uses fixed constants (JS_CENTER_X, JS_RANGE_X) rather than learning.
 * The learning code structure is kept for future enhancement.
 *
 * @param js Pointer to joystick HAL structure to initialize
 */
void joystick_hal_init(joystick_hal_t *js)
{
    // Raw ADC values (12-bit: 0-4095 typical range)
    js->raw_x = 0;
    js->raw_y = 0;

    // Calibration parameters (for auto-learning mode)
    js->center_x = 0;        // Center point X (typically ~2048 for 12-bit ADC)
    js->center_y = 0;        // Center point Y
    js->range_x = 1;         // Full-scale range X (prevents divide-by-zero)
    js->range_y = 1;         // Full-scale range Y

    // Normalized output values [-1.0, +1.0]
    js->norm_x = 0;
    js->norm_y = 0;

    // Deadband threshold (5% = ignore values within ±0.05 of center)
    js->deadband = 0.05f;

    // Auto-calibration parameters
    js->samples_collected = 0;   // Count of samples collected
    js->samples_needed = 200;    // Learn center over first 200 samples (~2-5 sec)

    // Assign update function pointer (enables polymorphism)
    js->update = joystick_hal_update;
}

/**
 * @brief Update joystick state from raw ADC values
 *
 * Processing Pipeline:
 *
 * 1. CENTER CORRECTION
 *    - Subtracts the known center point from raw values
 *    - Converts 0-4095 ADC range to centered -2048 to +2047 range
 *    - Example: If center = 2048 and raw = 3000:
 *      dx = 3000 - 2048 = +952 (joystick pushed right)
 *
 * 2. NORMALIZATION
 *    - Divides by range to get -1.0 to +1.0 scale
 *    - Example: 952 / 2048 = 0.465 (46.5% of full deflection)
 *    - This makes subsequent math independent of ADC resolution
 *
 * 3. CLAMPING
 *    - Prevents values exceeding ±1.0 due to:
 *      * ADC noise spikes
 *      * Mechanical over-travel
 *      * Calibration errors
 *
 * 4. ASYMMETRIC DEADBAND
 *    - X-axis (steering): 15% deadband (reduces twitchy turns)
 *    - Y-axis (throttle): 8% deadband (preserves speed sensitivity)
 *    - Different axes have different ergonomic requirements!
 *
 * Why Asymmetric Deadband?
 * - Steering (X): Users rest thumb on stick, causing drift
 *   Large deadband prevents unwanted turning
 * - Throttle (Y): Users actively control speed
 *   Small deadband preserves fine speed control
 *
 * Fixed Calibration Constants:
 * This implementation uses compile-time constants rather than auto-learning:
 * - JS_CENTER_X, JS_CENTER_Y: Measured joystick center (typically 1020-2048)
 * - JS_RANGE_X, JS_RANGE_Y: Full deflection range (typically 1000-2000)
 *
 * @param js Pointer to joystick HAL structure
 * @param x_raw Raw ADC value from X-axis (typically 0-4095 for 12-bit ADC)
 * @param y_raw Raw ADC value from Y-axis (typically 0-4095 for 12-bit ADC)
 *
 * @note Results stored in js->norm_x and js->norm_y (range: -1.0 to +1.0)
 */
void joystick_hal_update(joystick_hal_t *js, int32_t x_raw, int32_t y_raw)
{
    // Store raw values for debugging/telemetry
    js->raw_x = x_raw;
    js->raw_y = y_raw;

    // ═══════════════════════════════════════════════════════════
    // STEP 1: Center Correction
    // ═══════════════════════════════════════════════════════════
    // Convert raw ADC values to centered coordinates
    // Example: If JS_CENTER_X = 2048 (typical for 12-bit ADC):
    //   - Raw = 2048 → dx = 0 (centered)
    //   - Raw = 4095 → dx = +2047 (full right)
    //   - Raw = 0    → dx = -2048 (full left)
    float dx = (float)x_raw - JS_CENTER_X;
    float dy = (float)y_raw - JS_CENTER_Y;

    // ═══════════════════════════════════════════════════════════
    // STEP 2: Normalization to [-1, +1] Range
    // ═══════════════════════════════════════════════════════════
    // Divide by range to get unit-scale values
    // Example: If JS_RANGE_X = 2048 and dx = 1024:
    //   nx = 1024 / 2048 = 0.5 (50% deflection to right)
    float nx = dx / JS_RANGE_X;
    float ny = dy / JS_RANGE_Y;

    // ═══════════════════════════════════════════════════════════
    // STEP 3: Clamping to Valid Range
    // ═══════════════════════════════════════════════════════════
    // Prevent values outside [-1, +1] due to:
    // - Calibration errors (center/range not perfectly accurate)
    // - ADC noise (random fluctuations)
    // - Mechanical issues (stick pushed past normal limits)
    if (nx > 1.0f) nx = 1.0f;
    if (nx < -1.0f) nx = -1.0f;
    if (ny > 1.0f) ny = 1.0f;
    if (ny < -1.0f) ny = -1.0f;

    // ═══════════════════════════════════════════════════════════
    // STEP 4: Asymmetric Deadband Application
    // ═══════════════════════════════════════════════════════════
    // Different deadband thresholds for X and Y axes

    const float deadband_x = 0.10f;  // 15% steering deadband (wide zone) 0.15
    const float deadband_y = 0.08f;  // 8% throttle deadband (narrow zone) 0.08

    // Why 15% for X (steering)?
    // - Steering is very sensitive to drift
    // - Users rest their thumb on the stick
    // - Wide deadband prevents unwanted turning
    // - Sacrifice: Slightly delayed turn response (acceptable trade-off)

    // Why 8% for Y (throttle)?
    // - Speed control needs precision
    // - Users actively move stick for speed changes
    // - Narrow deadband preserves fine speed control
    // - Benefit: Smooth acceleration/deceleration

    js->norm_x = (fabsf(nx) < deadband_x) ? 0.0f : nx;
    js->norm_y = (fabsf(ny) < deadband_y) ? 0.0f : ny;

    // Result: Clean, drift-free normalized values ready for mixing
}

/**
 * @brief Clamp a float to a specified range
 *
 * Simple utility for range limiting. Equivalent to:
 *   return fminf(fmaxf(val, min), max);
 *
 * But written explicitly for clarity and to avoid extra function calls.
 *
 * @param val Value to clamp
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @return Clamped value within [min, max]
 */
static float clampf(float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}

/**
 * @brief Convert joystick X/Y coordinates to differential drive PWM values
 *
 * This is the "magic" function that makes intuitive robot control work.
 * It implements a sophisticated mixing algorithm with:
 *
 * 1. NON-LINEAR RESPONSE CURVE
 *    Uses cubic (x³) shaping for steering input
 *    - Near center: x³ is very small → gentle arcs
 *    - Near edges: x³ approaches ±1 → sharp turns
 *    - Eliminates the "twitchy" feeling of linear steering
 *
 * 2. DIFFERENTIAL MIXING
 *    Classic tank/skid-steer control:
 *    - Y (forward/back) applied equally to both motors
 *    - X (steering) added to one side, subtracted from other
 *    - Result: Natural feeling arcs and spins
 *
 * 3. ARC LIMITING
 *    Prevents overly aggressive turns that:
 *    - Cause tire slippage
 *    - Draw excessive current
 *    - Feel "snappy" and uncontrollable
 *
 * 4. PWM SCALING
 *    Converts normalized [-1, +1] to motor PWM range [0, 8191]
 *
 * Mathematical Breakdown:
 *
 * Given joystick inputs:
 *   x ∈ [-1, +1] (left/right, steering)
 *   y ∈ [-1, +1] (forward/back, throttle)
 *
 * Step 1: Shape steering with cubic curve
 *   x_shaped = x³
 *
 * Step 2: Apply steering gain
 *   x_scaled = k × x_shaped  (where k = 0.9)
 *
 * Step 3: Differential mix
 *   L = y + x_scaled  (left motor)
 *   R = y - x_scaled  (right motor)
 *
 * Step 4: Limit turning aggression
 *   diff = |L - R|
 *   if diff > max_diff: scale both down proportionally
 *
 * Step 5: Clamp to valid range
 *   L, R ∈ [-1, +1]
 *
 * Step 6: Scale to PWM
 *   pwm_left = L × 8190
 *   pwm_right = R × 8190
 *
 * Example Scenarios:
 *
 * A. Full forward (x=0, y=1):
 *    x_shaped = 0, L = 1, R = 1
 *    → Both motors full forward, straight line
 *
 * B. Gentle right arc (x=0.3, y=0.8):
 *    x_shaped = 0.027, x_scaled = 0.024
 *    L = 0.824, R = 0.776
 *    → Right motor slightly slower, smooth arc
 *
 * C. Sharp right turn (x=0.8, y=0.3):
 *    x_shaped = 0.512, x_scaled = 0.461
 *    L = 0.761, R = -0.161
 *    → Right motor reverses, tight turn
 *
 * D. Spin in place (x=1, y=0):
 *    x_shaped = 1, x_scaled = 0.9
 *    L = 0.9, R = -0.9
 *    → Equal opposite speeds, pivot turn
 *
 * @param x Normalized X input [-1.0, +1.0] (left = negative, right = positive)
 * @param y Normalized Y input [-1.0, +1.0] (back = negative, forward = positive)
 * @param pwm_left Output: Left motor PWM [-8190, +8190]
 * @param pwm_right Output: Right motor PWM [-8190, +8190]
 *
 * @note PWM range is ±8190 (not ±8191) to prevent overflow in some motor drivers
 */
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // ═══════════════════════════════════════════════════════════
    // STEP 1: Non-linear Steering Curve (Cubic Expo)
    // ═══════════════════════════════════════════════════════════
    float x_shaped = x * x * x;

    // Cubic (x³) Response Curve Analysis:
    //
    // Input  | Output | Effect
    // -------|--------|----------------------------------
    //  0.0   |  0.000 | No steering (centered)
    //  0.1   |  0.001 | Nearly imperceptible turn
    //  0.2   |  0.008 | Very gentle arc
    //  0.3   |  0.027 | Mild turn
    //  0.5   |  0.125 | Moderate arc
    //  0.7   |  0.343 | Strong turn
    //  0.9   |  0.729 | Sharp turn
    //  1.0   |  1.000 | Maximum steering
    //
    // Why Cubic?
    // - Linear (x): Too sensitive near center, feels "twitchy"
    // - Quadratic (x²): Better, but loses sign information
    // - Cubic (x³): Perfect! Smooth near center, responsive at edges
    // - Higher powers (x⁵, x⁷): Too aggressive, "dead zone" too large
    //
    // Alternative curves (commented out):
    //   x * fabsf(x) = x²·sign(x) → Quadratic expo (softer than cubic)
    //   x³ × 1.2 → Cubic with gain (more aggressive turning)

    // ═══════════════════════════════════════════════════════════
    // STEP 2: Steering Gain
    // ═══════════════════════════════════════════════════════════
    const float k = 0.8f; //0.9

    // Steering gain (k) controls turn aggressiveness:
    //
    // k value | Effect
    // --------|--------------------------------------------
    //   0.3   | Very soft, wide arcs only
    //   0.5   | Gentle steering, limited spin capability
    //   0.7   | Moderate, good for racing
    //   0.9   | Aggressive, strong spins (YOUR CHOICE)
    //   1.2   | Very aggressive, can be hard to control
    //
    // Your choice of 0.9 provides:
    // - Strong turning authority (can do 360° spins)
    // - Still controllable (not too "snappy")
    // - Good balance for general-purpose robot

    // ═══════════════════════════════════════════════════════════
    // STEP 3: Differential Mixing
    // ═══════════════════════════════════════════════════════════
    float L0 = y + k * x_shaped;
    float R0 = y - k * x_shaped;

    // Classic Tank Drive Formula:
    // - Both motors get the Y (throttle) component equally
    // - Steering component is ADDED to left, SUBTRACTED from right
    // - This creates the speed differential that causes turning
    //
    // Physics: When left motor spins faster than right,
    // the robot rotates clockwise (turns right)

    // ═══════════════════════════════════════════════════════════
    // STEP 4: Arc Limiting (Differential Limiter)
    // ═══════════════════════════════════════════════════════════
    float diff = fabsf(L0 - R0);
    float max_diff = 1.35f; //1.7

    // Why Limit the Differential?
    //
    // Problem: Without limiting, extreme joystick inputs can create
    // huge speed differences. Example:
    //   y = 0.5, x = 1.0 → L = 1.4, R = -0.4
    //   Both values get clamped: L = 1.0, R = -0.4
    //   This is NOT what we wanted! The ratio is destroyed.
    //
    // Solution: Limit |L - R| to max_diff BEFORE clamping
    //   If diff > max_diff: scale both L and R proportionally
    //   This preserves the ratio while preventing excessive turn rates
    //
    // max_diff values:
    //   1.0 → Very limited turning (60% of max)
    //   1.2 → Conservative (used previously, felt too weak)
    //   1.7 → Aggressive (YOUR CHOICE, allows ~85% max turn)
    //   2.0 → No limiting (can cause sharp, jarring movements)
    //
    // Your choice of 1.7 allows strong spins while keeping smooth arcs

    if (diff > max_diff) {
        float scale = max_diff / diff;
        L0 *= scale;
        R0 *= scale;
    }

    // Example: If diff = 2.0 and max_diff = 1.7:
    //   scale = 1.7 / 2.0 = 0.85
    //   Both L0 and R0 are multiplied by 0.85
    //   New diff = 1.7 (exactly at the limit)
    //   The SHAPE of the turn is preserved, just scaled down

    // ═══════════════════════════════════════════════════════════
    // STEP 5: Final Clamping
    // ═══════════════════════════════════════════════════════════
    // Ensure values are in valid range for PWM conversion
    // Note: No normalization here! We already scaled via diff limiting
    if (L0 > 1.0f) L0 = 1.0f;
    if (L0 < -1.0f) L0 = -1.0f;
    if (R0 > 1.0f) R0 = 1.0f;
    if (R0 < -1.0f) R0 = -1.0f;

    // ═══════════════════════════════════════════════════════════
    // STEP 6: Scale to PWM Range
    // ═══════════════════════════════════════════════════════════
    // Convert normalized [-1, +1] to motor controller PWM [0, 8191]
    // Using 8190 instead of 8191 to avoid potential overflow in some drivers
    *pwm_left  = (int)(L0 * 8190.0f);
    *pwm_right = (int)(R0 * 8190.0f);

    // Final output range: -8190 to +8190
    // - Positive = forward
    // - Negative = reverse
    // - Magnitude = speed (0 = stop, 8190 = full speed)
}
