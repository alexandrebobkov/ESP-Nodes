#include "control_task.h"
#include "joystick.h"
#include "esp_log.h"

static const char *TAG = "CONTROL";

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
} control_context_t;

static void control_task(void *arg) {
    control_context_t *ctx = (control_context_t *)arg;
    int pwm_left, pwm_right;

    ESP_LOGI(TAG, "Control task started");

    while (1) {
        // Get joystick values from ESP-NOW
        int x = ctx->espnow->last_data.x_axis;
        int y = ctx->espnow->last_data.y_axis;

        // Apply joystick mixing algorithm
        joystick_mix(y, x, &pwm_left, &pwm_right);

        // Update motors
        motor_set_pwm(ctx->motors, pwm_left, pwm_right);

        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz control rate
    }
}

void control_task_start(motor_system_t *motors, espnow_system_t *espnow) {
    static control_context_t ctx;
    ctx.motors = motors;
    ctx.espnow = espnow;

    xTaskCreate(control_task, "control", 2048, &ctx, 15, NULL);
    ESP_LOGI(TAG, "Control task created");
}
