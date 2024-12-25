#ifndef RC_H
#define RC_H

#define ADC_CHNL ADC_CHANNEL_1

#include "esp_adc/adc_oneshot.h"

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

#endif