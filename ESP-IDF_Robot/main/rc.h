#ifndef RC_H
#define RC_H

#define ADC_CHNL ADC_CHANNEL_1

#include "esp_adc/adc_oneshot.h"

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

static esp_err_t rc_adc_init (void) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_CHNL,
    };
    return adc_oneshot_new_unit(&init_config1, &adc1_handle);
}

#endif