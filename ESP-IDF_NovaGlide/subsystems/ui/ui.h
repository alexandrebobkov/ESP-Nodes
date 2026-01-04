#ifndef UI_H
#define UI_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define BLINK_GPIO 10
#define PUSH_BTN_GPIO 8
#define NAV_BTN_GPIO 8

// Forward declaration
typedef struct ui_system_t ui_system_t;

// Struct definition
struct ui_system_t {
    uint8_t led_state;

    void (*update)(ui_system_t *self, TickType_t now);
};

void ui_system_init(ui_system_t *sys);

#endif
