#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

static int x, y;
static adc_oneshot_unit_handle_t adc_xy_handle; 

static esp_err_t joystick_adc_init() {
    adc_oneshot_unit_init_cfg_t adc_init_config_xy = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config_xy, &adc_xy_handle));

    /*adc_oneshot_chan_cfg_t config_x = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        .atten = ADC_ATTEN_DB_11,

    };
    adc_oneshot_chan_cfg_t config_y = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        .atten = ADC_ATTEN_DB_11,

    };*/
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_xy_handle, ADC1_CHANNEL_0, &config_x));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_xy_handle, ADC1_CHANNEL_1, &config_y));

    return ESP_OK;
}

static void joystick_show_raw_xy() {

    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC1_CHANNEL_0, &x));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC1_CHANNEL_1, &y));
    ESP_LOGI("(x,y)", "( %d, %d )", x, y);
}

static void get_joystick_xy(int* x_axis, int* y_axis) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC1_CHANNEL_0, &x_axis));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC1_CHANNEL_1, &y_axis));
}

static void joystick_task(void *arg) {

    while (true) {
        joystick_show_raw_xy();
        vTaskDelay (750 / portTICK_PERIOD_MS);
    }
}

#endif