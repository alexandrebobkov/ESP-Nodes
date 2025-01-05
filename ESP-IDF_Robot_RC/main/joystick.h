#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

static unsigned uint8_t x = 0, y = 0;
adc_oneshot_unit_handle_t adc1_x_handle;

static esp_err_t joystick_adc_init() {
    adc_oneshot_unit_init_cgf_t adc_init_config1 = {
        .unit_id = ADC_UNIT_1,
    }
}

static void joystick_get_raw_xy() {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_x_handle, ADC1_CHANNEL_0, &x));
}

#endif