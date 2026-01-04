#include "ui.h"
#include "esp_log.h"
#include "freertos/queue.h"

static const char *TAG = "UI";

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task(void *arg) {
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Button GPIO[%lu] pressed, val: %d",
                     io_num, gpio_get_level(io_num));
        }
    }
}

static void ui_update_impl(ui_system_t *self, TickType_t now) {
    static TickType_t last_blink = 0;

    // Blink LED every 500ms
    if ((now - last_blink) >= pdMS_TO_TICKS(500)) {
        self->led_state = !self->led_state;
        gpio_set_level(BLINK_GPIO, self->led_state);
        last_blink = now;
    }
}

void ui_system_init(ui_system_t *sys) {
    sys->led_state = 0;
    sys->update = ui_update_impl;

    // Configure LED
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Configure buttons
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << PUSH_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(PUSH_BTN_GPIO, gpio_isr_handler, (void *)PUSH_BTN_GPIO);

    ESP_LOGI(TAG, "UI system initialized");
}
