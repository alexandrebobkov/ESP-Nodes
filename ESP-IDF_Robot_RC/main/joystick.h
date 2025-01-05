#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

static unsigned int x, y;
adc_oneshot_unit_handle_t adc1_x_handle, adc1_y_handle;

static esp_err_t joystick_adc_init() {
    adc_oneshot_unit_init_cfg_t adc_init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config1, &adc1_x_handle));
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config1, &adc1_y_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        .atten = ADC_ATTEN_DB_12,

    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_x_handle, ADC1_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_y_handle, ADC1_CHANNEL_1, &config));

    return ESP_OK;
}

static void joystick_get_raw_xy() {

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_x_handle, ADC1_CHANNEL_0, x));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_y_handle, ADC1_CHANNEL_1, y));

    ESP_LOGI("(x,y)", "( %i, %i)", x, y);
}

static void joystick_task(void *arg) {

    while (true) {
        joystick_get_raw_xy();
        vTaskDelay (750 / portTICK_PERIOD_MS);
    }
}

#endif