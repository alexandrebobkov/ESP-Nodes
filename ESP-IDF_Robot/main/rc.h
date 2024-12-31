#ifndef RC_H
#define RC_H

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "controls.h"

#define ADC_CHNL            ADC_CHANNEL_1
#define ADC_ATTEN           ADC_ATTEN_DB_11
#define ADC1_CHAN0          ADC1_CHANNEL_0
#define ADC1_CHAN1          ADC1_CHANNEL_1

//static const char *TAG = "ESP IDF Robot"
struct motors_rpm m;


static int adc_raw[2][10];
static int voltage[2][10];
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
adc_cali_handle_t adc1_cali_chan0_handle, adc1_cali_chan1_handle;

adc_oneshot_unit_handle_t adc1_handle;
bool do_calibration1_chan0, do_calibration1_chan1;

static int interpolate_raw_val (int raw);
static int rescale_raw_val (int raw);

static esp_err_t rc_adc_init (void) {
    
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK( adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,//ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//
    
    adc1_cali_chan0_handle = NULL;
    adc1_cali_chan1_handle = NULL;
    do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN0, ADC_ATTEN, &adc1_cali_chan0_handle);
    do_calibration1_chan1 = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN1, ADC_ATTEN, &adc1_cali_chan1_handle);

    return ESP_OK;
}

static void rc_get_raw_data() {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[0][0]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw[0][1]));
    ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN0, adc_raw[0][0]);
    ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1, adc_raw[0][1]);
    ESP_LOGI("Joystick L/R", "Position: %d", rescale_raw_val(adc_raw[0][0]));
    ESP_LOGI("Joystick F", "Position: %d", rescale_raw_val(adc_raw[0][1]));

    int x = rescale_raw_val(adc_raw[0][0]);
    int y = rescale_raw_val(adc_raw[0][1]);
    if ((x > 0 && x < 500) && (y > 500))
        ESP_LOGW("RC", "FORWARD");
    else if ((x > 0 && x < 500) && (y < -200))
        ESP_LOGW("RC", "REVERSE");
    else if ((y < 0 && y > -200) && (x < -1000))
        ESP_LOGW("RC", "LEFT");
    else if ((y < 0 && y > -200) && (x > 1000))
        ESP_LOGW("RC", "RIGHT");
    else
        ESP_LOGW("RC", "STAND STILL");
    
    /*// Cut-off readings that have values less than 700, i.e. when joystick is centered
    if (rescale_raw_val(adc_raw[0][1]) < -500 && rescale_raw_val(adc_raw[0][0]) < 500) {
            ESP_LOGW("RC", "REVERSE");
    }
    // FORWARD
    else if (rescale_raw_val(adc_raw[0][1]) >= 500 && rescale_raw_val(adc_raw[0][0]) < 500) {
        m.motor1_rpm_pcm = rescale_raw_val(adc_raw[0][1]);
        ESP_LOGW("RC", "FORWARD");
        //m.motor2_rpm_pcm = rescale_raw_val(adc_raw[0][1]);
    }
    else {
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        ESP_LOGW("RC", "STAD STILL");
    }
    */
    // LEFT or RIGHT
    // RIGHT
    /*if (rescale_raw_val(adc_raw[0][0]) >= 4000) {
        m.motor1_rpm_pcm = rescale_raw_val(adc_raw[0][0]);
    }
    // LEFT
    else if (rescale_raw_val(adc_raw[0][0]) < 0) {
        m.motor2_rpm_pcm = -rescale_raw_val(adc_raw[0][0]);
    }
    else {
        m.motor1_rpm_pcm = 0;
    }*/
    
    // Cut-off power if joystick is nearly center
    /*if (rescale_raw_val(adc_raw[0][0]) < 4000)
        m.motor1_rpm_pcm = 0;
    if (rescale_raw_val(adc_raw[0][1]) < 4000)
        m.motor2_rpm_pcm = 0;*/
    /*
    if (rescale_raw_val(adc_raw[0][0]) >= 4000)
        m.motor1_rpm_pcm = rescale_raw_val(adc_raw[0][0]);
    else
        m.motor1_rpm_pcm = 0;
    if (rescale_raw_val(adc_raw[0][1]) >= 4000)
        m.motor2_rpm_pcm = rescale_raw_val(adc_raw[0][1]);
    else
        m.motor2_rpm_pcm = 0;
    */

    ESP_LOGI("PWM", "Motor 1 PWM: %d", m.motor1_rpm_pcm);
    ESP_LOGI("PWM", "Motor 2 PWM: %d", m.motor2_rpm_pcm);

    if (do_calibration1_chan0) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN0, voltage[0][0]);
    }
    if (do_calibration1_chan1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
        ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN1, voltage[0][1]);
    }

    //Tear Down
    /*ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    if (do_calibration1_chan1) {
        adc_calibration_deinit(adc1_cali_chan1_handle);
    }*/
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("ESP IDF Robot", "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("ESP IDF Robot", "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI("ESP IDF Robot", "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW("ESP IDF Robot", "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE("ESP IDF Robot", "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI("ESP IDF Robot", "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI("ESP IDF Robot", "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

#endif