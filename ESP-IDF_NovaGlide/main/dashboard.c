// dashboard.c
#include "dashboard.h"
#include <stdio.h>
#include "esp_log.h"

// ANSI escape codes for terminal control
#define CLEAR_SCREEN "\033[2J"
#define CURSOR_HOME "\033[H"
#define CURSOR_HIDE "\033[?25l"
#define CURSOR_SHOW "\033[?25h"
#define COLOR_RESET "\033[0m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_CYAN "\033[36m"
#define COLOR_RED "\033[31m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define BOLD "\033[1m"

static void draw_box(const char *title) {
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║ " COLOR_CYAN BOLD "%-58s" COLOR_RESET " ║\n", title);
    printf("╠════════════════════════════════════════════════════════════╣\n");
}

static void draw_line(const char *label, const char *value, const char *color) {
    printf("║ " BOLD "%-20s" COLOR_RESET " : %s%-33s" COLOR_RESET "   ║\n", label, color, value);
}

static void draw_separator() {
    printf("╠════════════════════════════════════════════════════════════╣\n");
}

static void draw_bottom() {
    printf("╚════════════════════════════════════════════════════════════╝\n");
}

static void dashboard_task(void *arg) {
    dashboard_context_t *ctx = (dashboard_context_t *)arg;
    char buffer[50];

    // Hide cursor for cleaner display
    printf(CURSOR_HIDE);

    // Print help text ONCE before the loop
    printf("\n" COLOR_YELLOW "Dashboard Mode: Press Ctrl+] to exit monitor" COLOR_RESET "\n");
    vTaskDelay(pdMS_TO_TICKS(2000));  // Give user time to read


    while (1) {
        // Clear screen and move cursor to home
        printf(CLEAR_SCREEN CURSOR_HOME);

        // ========== HEADER ==========
        draw_box("ESP32-C3 ROBOT CONTROL DASHBOARD");

        // ========== ESP-NOW SECTION ==========
        snprintf(buffer, sizeof(buffer), "%d", ctx->espnow->last_data.x_axis);
        draw_line("Joystick X", buffer, COLOR_GREEN);

        snprintf(buffer, sizeof(buffer), "%d", ctx->espnow->last_data.y_axis);
        draw_line("Joystick Y", buffer, COLOR_GREEN);

        draw_separator();

        // ========== MOTORS SECTION ==========
        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->left_pwm);
        draw_line("PWM Left", buffer, COLOR_YELLOW);

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->right_pwm);
        draw_line("PWM Right", buffer, COLOR_YELLOW);

        draw_separator();

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->motor1_rpm_pcm);
        draw_line("Motor 1 (L-Fwd)", buffer,
                  ctx->motors->motor1_rpm_pcm > 0 ? COLOR_GREEN : COLOR_RESET);

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->motor2_rpm_pcm);
        draw_line("Motor 2 (R-Fwd)", buffer,
                  ctx->motors->motor2_rpm_pcm > 0 ? COLOR_GREEN : COLOR_RESET);

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->motor3_rpm_pcm);
        draw_line("Motor 3 (L-Rev)", buffer,
                  ctx->motors->motor3_rpm_pcm > 0 ? COLOR_RED : COLOR_RESET);

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->motor4_rpm_pcm);
        draw_line("Motor 4 (R-Rev)", buffer,
                  ctx->motors->motor4_rpm_pcm > 0 ? COLOR_RED : COLOR_RESET);

        draw_separator();

        // ========== SENSORS SECTION ==========
        snprintf(buffer, sizeof(buffer), "%.2f °C", ctx->temp->temperature);
        draw_line("Temperature", buffer, COLOR_CYAN);

        snprintf(buffer, sizeof(buffer), "%.2f V", ctx->ina->bus_voltage);
        draw_line("Battery Voltage", buffer,
                  ctx->ina->bus_voltage > 7.0 ? COLOR_GREEN : COLOR_RED);

        snprintf(buffer, sizeof(buffer), "%.2f mA", ctx->ina->current * 1000.0f);
        draw_line("Current", buffer, COLOR_MAGENTA);

        snprintf(buffer, sizeof(buffer), "%.2f mW", ctx->ina->power * 1000.0f);
        draw_line("Power", buffer, COLOR_BLUE);

        draw_separator();

        sprintf(buffer, sizeof(buffer), "%.2f cm", ctx->ultrasonic->distance);
        draw_line("Distance", buffer, COLOR_BLUE);

        draw_bottom();

        // Status bar at bottom
        //printf("\n" COLOR_YELLOW "Press Ctrl+] to exit monitor" COLOR_RESET "\n");

        // Update every 200ms for smooth display
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Show cursor on exit (won't actually run due to while loop)
    printf(CURSOR_SHOW);
}

void dashboard_task_start(dashboard_context_t *ctx) {
    xTaskCreate(dashboard_task, "dashboard", 4096, ctx, 6, NULL);
    ESP_LOGI("DASHBOARD", "Dashboard display started");
}
