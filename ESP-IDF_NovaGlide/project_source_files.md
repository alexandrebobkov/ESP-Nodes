# Project Source Archive

Generated automatically for analysis.

## File: `./main/system_init.c`

```text
#include "system_init.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "SYSTEM_INIT";

void system_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "System initialization complete");
}

```

## File: `./main/dashboard.c`

```text
// dashboard.c
#include "dashboard.h"
#include "ultrasonic_sensor.h"
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

        snprintf(buffer, sizeof(buffer), "%.2f cm", ctx->ultrasonic->distance_cm / 10.0f);
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

```

## File: `./main/CMakeLists.txt`

```text
idf_component_register(
    SRCS "ESP-IDF_NovaGlide.c"
         "system_init.c"
         "scheduler.c"
         "control_task.c"
         "dashboard.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_netif
        nvs_flash
        motors
        adc
        wifi_sys
        mqtt_sys
        espnow_sys
        sensors
        controls
        ui
        i2c_bus)

```

## File: `./main/scheduler.c`

```text
#include "scheduler.h"
#include "esp_log.h"

static const char *TAG = "SCHEDULER";

static void scheduler_task(void *arg) {
    scheduler_t *sched = (scheduler_t *)arg;

    ESP_LOGI(TAG, "Scheduler task started");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Update all subsystems
        if (sched->motors && sched->motors->update) {
            sched->motors->update(sched->motors, now);
        }
        if (sched->adc && sched->adc->update) {
            sched->adc->update(sched->adc, now);
        }
        if (sched->temp && sched->temp->update) {
            sched->temp->update(sched->temp, now);
        }
        if (sched->ina && sched->ina->update) {
            sched->ina->update(sched->ina, now);
        }
        if (sched->ultra && sched->ultra->update) {
            sched->ultra->update(sched->ultra, now);
        }
        if (sched->mqtt && sched->mqtt->update) {
            sched->mqtt->update(sched->mqtt, now);
        }
        if (sched->espnow && sched->espnow->update) {
            sched->espnow->update(sched->espnow, now);
        }
        if (sched->ui && sched->ui->update) {
            sched->ui->update(sched->ui, now);
        }

        vTaskDelay(pdMS_TO_TICKS(250));  // 20Hz update rate
    }
}

void scheduler_init(scheduler_t *sched) {
    ESP_LOGI(TAG, "Scheduler initialized");
}

void scheduler_start(scheduler_t *sched) {
    xTaskCreate(scheduler_task, "scheduler", 8192, sched, 10, NULL);
    ESP_LOGI(TAG, "Scheduler started");
}

```

## File: `./main/control_task.c`

```text
#include "control_task.h"
#include "joystick.h"
#include "esp_log.h"

static const char *TAG = "CONTROL";

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
} control_context_t;

static joystick_hal_t js;

static void control_task(void *arg) {
    control_context_t *ctx = (control_context_t *)arg;
    int pwm_left = 0;
    int pwm_right = 0;

    ESP_LOGI(TAG, "Control task started");

    // Initialize joystick HAL
    joystick_hal_init(&js);

    while (1) {
        // 1. Read raw joystick values from ESP-NOW
        int32_t rc_x = ctx->espnow->last_data.x_axis;
        int32_t rc_y = ctx->espnow->last_data.y_axis;

        // 2. Update joystick HAL (auto-calibration + normalization)
        js.update(&js, rc_x, rc_y);

        // 3. Mix normalized joystick values into motor PWM
        joystick_mix(js.norm_y, js.norm_x, &pwm_left, &pwm_right);

        // 4. Apply PWM to motors
        update_motors_pwm(ctx->motors, pwm_left, pwm_right);

        // 5. Debug output
        ESP_LOGI(TAG,
                 "RC raw=(%ld,%ld) norm=(%.2f,%.2f) PWM(L,R)=(%d,%d)",
                 (long)rc_x, (long)rc_y,
                 js.norm_x, js.norm_y,
                 pwm_left, pwm_right);

        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz control loop
    }
}

void control_task_start(motor_system_t *motors, espnow_system_t *espnow) {
    static control_context_t ctx;
    ctx.motors = motors;
    ctx.espnow = espnow;

    xTaskCreate(control_task, "control", 4096, &ctx, 15, NULL);
    ESP_LOGI(TAG, "Control task created");
}

```

## File: `./main/ESP-IDF_NovaGlide.c`

```text
#include "esp_log_level.h"
#include "espnow_sys.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include "system_init.h"
#include "scheduler.h"
#include "control_task.h"

// Subsystems
#include "motors.h"
#include "adc.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"
#include "wifi_sys.h"
#include "espnow_sys.h"
#include "mqtt_sys.h"
#include "ui.h"
#include "dashboard.h"
#include "i2c_bus.h"

// Telemetry bridge context
typedef struct {
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    motor_system_t *motors;
    mqtt_system_t *mqtt;
    ultrasonic_system_t *ultrasonic;
} telemetry_context_t;

// Task to bridge sensor data to MQTT
static void telemetry_bridge_task(void *arg) {
    telemetry_context_t *ctx = (telemetry_context_t *)arg;

    while (1) {
        // Update MQTT with latest sensor readings
        mqtt_update_temp(ctx->mqtt, ctx->temp->temperature);
        mqtt_update_battery(ctx->mqtt, ctx->ina->bus_voltage);
        mqtt_update_current(ctx->mqtt, ctx->ina->current * 1000.0f);
        mqtt_update_power(ctx->mqtt, ctx->ina->power);
        mqtt_update_pwm(ctx->mqtt, ctx->motors->left_pwm, ctx->motors->right_pwm);
        mqtt_update_proximity(ctx->mqtt, ctx->ultrasonic->distance_cm / 10.0f);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Display task to show current joystick values
static void display_joystick_task(void *arg) {
    espnow_system_t *espnow = (espnow_system_t *)arg;

    while (1) {
        ESP_LOGI("DISPLAY", "╔════════════════════════════════════╗");
        ESP_LOGI("DISPLAY", "║   CURRENT JOYSTICK VALUES          ║");
        ESP_LOGI("DISPLAY", "║   X-axis: %-8d                     ║", espnow->last_data.x_axis);
        ESP_LOGI("DISPLAY", "║   Y-axis: %-8d                ║", espnow->last_data.y_axis);
        ESP_LOGI("DISPLAY", "╚════════════════════════════════════╝");

        vTaskDelay(pdMS_TO_TICKS(2000));  // Display every 2 seconds
    }
}

void app_main(void)
{
    // System-level initialization
    system_init();

    // Set log levels to WARNING or ERROR only
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("DASHBOARD", ESP_LOG_INFO);  // Allow dashboard
    esp_log_level_set("i2c.master", ESP_LOG_NONE);  // Suppress I2C NACK errors (normal when ultrasonic is too close)

    // Subsystem instances
    static motor_system_t motors;
    static adc_system_t adc;
    static temp_sensor_system_t temp;
    static ina219_system_t ina;
    static ultrasonic_system_t ultra;
    static mqtt_system_t mqtt;
    static espnow_system_t espnow;
    static ui_system_t ui;

    // Initialize WiFi first (needed for ESP-NOW and MQTT)
    wifi_system_init();

    // Initialize I2C bus FIRST
    ESP_ERROR_CHECK(i2c_bus_init());
    i2c_bus_scan();

    // Initialize all subsystems
    motor_system_init(&motors);
    adc_system_init(&adc);
    temp_sensor_system_init(&temp);
    ina219_system_init(&ina);
    ultrasonic_system_init(&ultra);
    espnow_system_init(&espnow);
    mqtt_system_init(&mqtt);
    ui_system_init(&ui);

    // Start display task (optional - uncomment if needed)
    // xTaskCreate(display_joystick_task, "display", 2048, &espnow, 4, NULL);

    // Start control task (joystick -> motors)
    control_task_start(&motors, &espnow);

    // Start dashboard display
    static dashboard_context_t dash_ctx = {
        .motors = &motors,
        .espnow = &espnow,
        .temp = &temp,
        .ina = &ina,
        .ultrasonic = &ultra
    };
    dashboard_task_start(&dash_ctx);

    // Create data bridge task for MQTT telemetry
    static telemetry_context_t telem_ctx = {
        .temp = &temp,
        .ina = &ina,
        .motors = &motors,
        .mqtt = &mqtt,
        .ultrasonic = &ultra
    };
    xTaskCreate(telemetry_bridge_task, "telemetry", 4096, &telem_ctx, 5, NULL);

    // Scheduler wiring
    static scheduler_t sched = {
        .motors = &motors,
        .adc = &adc,
        .temp = &temp,
        .ina = &ina,
        .ultra = &ultra,
        .mqtt = &mqtt,
        .espnow = &espnow,
        .ui = &ui
    };
    scheduler_init(&sched);
    scheduler_start(&sched);
}

```

## File: `./main/system_init.h`

```text
#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

void system_init(void);

#endif

```

## File: `./main/dashboard.h`

```text
// dashboard.h
#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "freertos/FreeRTOS.h"
#include "motors.h"
#include "espnow_sys.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    ultrasonic_system_t *ultrasonic;
} dashboard_context_t;

void dashboard_task_start(dashboard_context_t *ctx);

#endif

```

## File: `./main/control_task.h`

```text
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "motors.h"
#include "espnow_sys.h"

void control_task_start(motor_system_t *motors, espnow_system_t *espnow);

#endif

```

## File: `./main/scheduler.h`

```text
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"
#include "adc.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"
#include "mqtt_sys.h"
#include "espnow_sys.h"
#include "ui.h"

typedef struct {
    motor_system_t *motors;
    adc_system_t *adc;
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    ultrasonic_system_t *ultra;
    mqtt_system_t *mqtt;
    espnow_system_t *espnow;
    ui_system_t *ui;
} scheduler_t;

void scheduler_init(scheduler_t *sched);
void scheduler_start(scheduler_t *sched);

#endif

```

## File: `./dashboard.h`

```text
// dashboard.h
#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "freertos/FreeRTOS.h"
#include "motors.h"
#include "espnow_sys.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    ultrasonic_system_t *ultrasonic;
} dashboard_context_t;

void dashboard_task_start(dashboard_context_t *ctx);

#endif

```

## File: `./control_task.c`

```text
#include "control_task.h"
#include "joystick.h"
#include "esp_log.h"

static const char *TAG = "CONTROL";

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
} control_context_t;

static joystick_hal_t js;

static void control_task(void *arg) {
    control_context_t *ctx = (control_context_t *)arg;
    int pwm_left = 0;
    int pwm_right = 0;

    ESP_LOGI(TAG, "Control task started");

    // Initialize joystick HAL
    joystick_hal_init(&js);

    while (1) {
        // 1. Read raw joystick values from ESP-NOW
        int32_t rc_x = ctx->espnow->last_data.x_axis;
        int32_t rc_y = ctx->espnow->last_data.y_axis;

        // 2. Update joystick HAL (auto-calibration + normalization)
        js.update(&js, rc_x, rc_y);

        // 3. Mix normalized joystick values into motor PWM
        joystick_mix(js.norm_y, js.norm_x, &pwm_left, &pwm_right);

        // 4. Apply PWM to motors
        update_motors_pwm(ctx->motors, pwm_left, pwm_right);

        // 5. Debug output
        ESP_LOGI(TAG,
                 "RC raw=(%ld,%ld) norm=(%.2f,%.2f) PWM(L,R)=(%d,%d)",
                 (long)rc_x, (long)rc_y,
                 js.norm_x, js.norm_y,
                 pwm_left, pwm_right);

        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz control loop
    }
}

void control_task_start(motor_system_t *motors, espnow_system_t *espnow) {
    static control_context_t ctx;
    ctx.motors = motors;
    ctx.espnow = espnow;

    xTaskCreate(control_task, "control", 4096, &ctx, 15, NULL);
    ESP_LOGI(TAG, "Control task created");
}

```

## File: `./ESP-IDF_NovaGlide.c`

```text
#include "esp_log_level.h"
#include "espnow_sys.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include "system_init.h"
#include "scheduler.h"
#include "control_task.h"

// Subsystems
#include "motors.h"
#include "adc.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"
#include "wifi_sys.h"
#include "espnow_sys.h"
#include "mqtt_sys.h"
#include "ui.h"
#include "dashboard.h"
#include "i2c_bus.h"

// Telemetry bridge context
typedef struct {
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    motor_system_t *motors;
    mqtt_system_t *mqtt;
    ultrasonic_system_t *ultrasonic;
} telemetry_context_t;

// Task to bridge sensor data to MQTT
static void telemetry_bridge_task(void *arg) {
    telemetry_context_t *ctx = (telemetry_context_t *)arg;

    while (1) {
        // Update MQTT with latest sensor readings
        mqtt_update_temp(ctx->mqtt, ctx->temp->temperature);
        mqtt_update_battery(ctx->mqtt, ctx->ina->bus_voltage);
        mqtt_update_current(ctx->mqtt, ctx->ina->current * 1000.0f);
        mqtt_update_power(ctx->mqtt, ctx->ina->power);
        mqtt_update_pwm(ctx->mqtt, ctx->motors->left_pwm, ctx->motors->right_pwm);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Display task to show current joystick values
static void display_joystick_task(void *arg) {
    espnow_system_t *espnow = (espnow_system_t *)arg;

    while (1) {
        ESP_LOGI("DISPLAY", "╔════════════════════════════════════╗");
        ESP_LOGI("DISPLAY", "║   CURRENT JOYSTICK VALUES          ║");
        ESP_LOGI("DISPLAY", "║   X-axis: %-8d                ║", espnow->last_data.x_axis);
        ESP_LOGI("DISPLAY", "║   Y-axis: %-8d                ║", espnow->last_data.y_axis);
        ESP_LOGI("DISPLAY", "╚════════════════════════════════════╝");

        vTaskDelay(pdMS_TO_TICKS(2000));  // Display every 2 seconds
    }
}

void app_main(void)
{
    // System-level initialization
    system_init();

    // Set log levels to WARNING or ERROR only
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("DASHBOARD", ESP_LOG_INFO);  // Allow dashboard
    esp_log_level_set("i2c.master", ESP_LOG_NONE);  // Suppress I2C NACK errors (normal when ultrasonic is too close)

    // Subsystem instances
    static motor_system_t motors;
    static adc_system_t adc;
    static temp_sensor_system_t temp;
    static ina219_system_t ina;
    static ultrasonic_system_t ultra;
    static mqtt_system_t mqtt;
    static espnow_system_t espnow;
    static ui_system_t ui;

    // Initialize WiFi first (needed for ESP-NOW and MQTT)
    wifi_system_init();

    // Initialize I2C bus FIRST
    ESP_ERROR_CHECK(i2c_bus_init());
    i2c_bus_scan();

    // Initialize all subsystems
    motor_system_init(&motors);
    adc_system_init(&adc);
    temp_sensor_system_init(&temp);
    ina219_system_init(&ina);
    ultrasonic_system_init(&ultra);
    espnow_system_init(&espnow);
    mqtt_system_init(&mqtt);
    ui_system_init(&ui);

    // Start display task (optional - uncomment if needed)
    // xTaskCreate(display_joystick_task, "display", 2048, &espnow, 4, NULL);

    // Start control task (joystick -> motors)
    control_task_start(&motors, &espnow);

    // Start dashboard display
    static dashboard_context_t dash_ctx = {
        .motors = &motors,
        .espnow = &espnow,
        .temp = &temp,
        .ina = &ina,
        .ultrasonic = &ultra
    };
    dashboard_task_start(&dash_ctx);

    // Create data bridge task for MQTT telemetry
    static telemetry_context_t telem_ctx = {
        .temp = &temp,
        .ina = &ina,
        .motors = &motors,
        .mqtt = &mqtt,
        .ultrasonic = &ultra
    };
    xTaskCreate(telemetry_bridge_task, "telemetry", 4096, &telem_ctx, 5, NULL);

    // Scheduler wiring
    static scheduler_t sched = {
        .motors = &motors,
        .adc = &adc,
        .temp = &temp,
        .ina = &ina,
        .ultra = &ultra,
        .mqtt = &mqtt,
        .espnow = &espnow,
        .ui = &ui
    };
    scheduler_init(&sched);
    scheduler_start(&sched);
}

```

## File: `./CMakeLists.txt`

```text
cmake_minimum_required(VERSION 3.16)

# Add subsystems to component search path
set(EXTRA_COMPONENT_DIRS
    "subsystems/connectivity/wifi_sys"
    "subsystems/connectivity/mqtt_sys"
    "subsystems/connectivity/espnow_sys"
    "subsystems/motors"
    "subsystems/adc"
    "subsystems/sensors"
    "subsystems/controls"
    "subsystems/ui"
    "subsystems/i2c_bus")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP-IDF_NovaGlide)

```

## File: `./scheduler.h`

```text
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"
#include "adc.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"
#include "mqtt_sys.h"
#include "espnow_sys.h"
#include "ui.h"

typedef struct {
    motor_system_t *motors;
    adc_system_t *adc;
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    ultrasonic_system_t *ultra;
    mqtt_system_t *mqtt;
    espnow_system_t *espnow;
    ui_system_t *ui;
} scheduler_t;

void scheduler_init(scheduler_t *sched);
void scheduler_start(scheduler_t *sched);

#endif

```

## File: `./system_init.h`

```text
#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

void system_init(void);

#endif

```

## File: `./scan_code.py`

```text
import os

# Configure this to your project root
PROJECT_ROOT = "./"
#/home/alex/github/ESP-Nodes/ESP-IDF_NovaGlide"
OUTPUT_MD = "project_source_files.md"

# Folders to scan (relative to project root)
TARGET_FOLDERS = [
    "main",
]

# File extensions to include
SOURCE_EXTENSIONS = {
    ".c", ".h", ".cpp", ".hpp",
    ".py", ".txt", ".cmake",
}

# Special filenames to include even without extension
SPECIAL_FILES = {
    "CMakeLists.txt",
}

def is_source_file(filename):
    # Special cases
    if filename in SPECIAL_FILES:
        return True

    # Extension-based detection
    _, ext = os.path.splitext(filename)
    return ext in SOURCE_EXTENSIONS

def collect_sources(root):
    sources = []

    # Scan only selected folders
    for folder in TARGET_FOLDERS:
        full_path = os.path.join(root, folder)
        if not os.path.exists(full_path):
            continue

        for dirpath, _, filenames in os.walk(full_path):
            for f in filenames:
                if is_source_file(f):
                    sources.append(os.path.join(dirpath, f))

    # Also include top-level files
    for f in os.listdir(root):
        if is_source_file(f):
            sources.append(os.path.join(root, f))

    return sources

def write_markdown(sources, output_file):
    with open(output_file, "w", encoding="utf-8") as md:
        md.write("# Project Source Archive\n\n")
        md.write("Generated automatically for analysis.\n\n")

        for path in sources:
            md.write(f"## File: `{path}`\n\n")
            md.write("```text\n")

            try:
                with open(path, "r", encoding="utf-8", errors="replace") as src:
                    md.write(src.read())
            except Exception as e:
                md.write(f"[Error reading file: {e}]")

            md.write("\n```\n\n")

if __name__ == "__main__":
    sources = collect_sources(PROJECT_ROOT)
    write_markdown(sources, OUTPUT_MD)
    print(f"Markdown document generated: {OUTPUT_MD}")

```

## File: `./control_task.h`

```text
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "motors.h"
#include "espnow_sys.h"

void control_task_start(motor_system_t *motors, espnow_system_t *espnow);

#endif

```

## File: `./dashboard.c`

```text
// dashboard.c
#include "dashboard.h"
#include "ultrasonic_sensor.h"
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

        snprintf(buffer, sizeof(buffer), "%.2f cm", ctx->ultrasonic->distance_cm / 10.0f);
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

```

## File: `./scheduler.c`

```text
#include "scheduler.h"
#include "esp_log.h"

static const char *TAG = "SCHEDULER";

static void scheduler_task(void *arg) {
    scheduler_t *sched = (scheduler_t *)arg;

    ESP_LOGI(TAG, "Scheduler task started");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Update all subsystems
        if (sched->motors && sched->motors->update) {
            sched->motors->update(sched->motors, now);
        }
        if (sched->adc && sched->adc->update) {
            sched->adc->update(sched->adc, now);
        }
        if (sched->temp && sched->temp->update) {
            sched->temp->update(sched->temp, now);
        }
        if (sched->ina && sched->ina->update) {
            sched->ina->update(sched->ina, now);
        }
        if (sched->ultra && sched->ultra->update) {
            sched->ultra->update(sched->ultra, now);
        }
        if (sched->mqtt && sched->mqtt->update) {
            sched->mqtt->update(sched->mqtt, now);
        }
        if (sched->espnow && sched->espnow->update) {
            sched->espnow->update(sched->espnow, now);
        }
        if (sched->ui && sched->ui->update) {
            sched->ui->update(sched->ui, now);
        }

        vTaskDelay(pdMS_TO_TICKS(250));  // 20Hz update rate
    }
}

void scheduler_init(scheduler_t *sched) {
    ESP_LOGI(TAG, "Scheduler initialized");
}

void scheduler_start(scheduler_t *sched) {
    xTaskCreate(scheduler_task, "scheduler", 8192, sched, 10, NULL);
    ESP_LOGI(TAG, "Scheduler started");
}

```

## File: `./system_init.c`

```text
#include "system_init.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "SYSTEM_INIT";

void system_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "System initialization complete");
}

```

