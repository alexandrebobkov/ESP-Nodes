#include <esp_log.h>
#include <app_reset.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

//#define SYS_LED 21    // ESP32 S2 mini
#define SYS_LED 8   // ESP32 C3 SUPER mini

esp_err_t sys_status_set_gpio (const char *name, bool state) {
    gpio_set_level(SYS_LED, state);
    return ESP_OK;
}
void sys_status_init() {
    // Configure GPIO power states
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    //uint64_t pin_mask = (((uint64_t)1 << OUTPUT_GPIO_RED ) | ((uint64_t)1 << OUTPUT_GPIO_GREEN ) | ((uint64_t)1 << OUTPUT_GPIO_BLUE ));
    uint64_t pin_mask = ((uint64_t)1 << SYS_LED );
    io_conf.pin_bit_mask = pin_mask;
    
    /* Configure the GPIO */
    gpio_config(&io_conf);
    gpio_set_level(SYS_LED, true);
    //gpio_set_level(OUTPUT_GPIO_GREEN, false);
    //gpio_set_level(OUTPUT_GPIO_BLUE, false);
}