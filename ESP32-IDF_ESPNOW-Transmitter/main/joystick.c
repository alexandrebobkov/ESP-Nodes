#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"

adc_oneshot_unit_handle_t adc_xy_handle;
static int x, y; // Joystick x- and y- axis positions


esp_err_t joystick_adc_init(void) {
    adc_oneshot_unit_init_cfg_t adc_init_config_xy = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config_xy, &adc_xy_handle));

    adc_oneshot_chan_cfg_t config_x = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_chan_cfg_t config_y = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_xy_handle, ADC_CHANNEL_0, &config_x));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_xy_handle, ADC_CHANNEL_1, &config_y));

    return ESP_OK;
}

void joystick_show_raw_xy() {
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_0, &x));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_1, &y));
    ESP_LOGI("(x,y)", "( %d, %d )", x, y);
}

void get_joystick_xy(int *x_axis, int *y_axis) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_0, x_axis));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_1, y_axis));
}