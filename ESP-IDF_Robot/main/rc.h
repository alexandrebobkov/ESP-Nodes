#ifndef RC_H
#define RC_H

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "controls.h"
#include "config.h"

#define ADC_CHNL            ADC_CHANNEL_1
#define ADC_ATTEN           ADC_ATTEN_DB_11
#define ADC1_CHAN0          ADC1_CHANNEL_0
#define ADC1_CHAN1          ADC1_CHANNEL_1

//static const char *TAG = "ESP IDF Robot"
struct motors_rpm m;
static int adc_raw[2][10];
static int voltage[2][10];
static int s = 0, sample = 5, x = 0, y = 0, x_sum = 0, y_sum = 0;

bool do_calibration1_chan0, do_calibration1_chan1;

adc_cali_handle_t adc1_cali_chan0_handle, adc1_cali_chan1_handle;
adc_oneshot_unit_handle_t adc1_handle;
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
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

static int check_motor_pcm(int x) {
    int lim = 7440;
    if (x > lim)
        return 8190;//lim;
    else if (x < -lim)
        return -8190;//-lim;
    else
        return x;
}

// Update PWM based on received values
// IMPORTANT: x and y values correspod to the PWM!
static void update_pwm (int rc_x, int rc_y) {
    
    x = check_motor_pcm(rescale_raw_val(rc_x));
    y = check_motor_pcm(rescale_raw_val(rc_y));
    //ESP_LOGI("x,y", "( %d, %d ) [ %d, %d] ", rc_x, rc_y, x, y);

    /*if (s < sample) {
        x_sum += check_motor_pcm(rescale_raw_val(x));
        y_sum += check_motor_pcm(rescale_raw_val(y));
        s ++;
    }
    else if (s == sample) {
        //x = check_motor_pcm(rescale_raw_val(adc_raw[0][0]));
        //y = check_motor_pcm(rescale_raw_val(adc_raw[0][1]));
        x = x_sum / sample;
        y = y_sum / sample;
        s++;*/

    /*
    
    (+, +0)     (+, +)      (+0, +)
    (-, +)      (0, 0)      (+, -)
    (-, -0)     (-, -)      (-0, -)
    
    if (1024 < x < 2048 && 1024 < y < 2048) {}
    */

    // ADDED ON AUG 6, 2025: to be tested!
    // CONTINOUS UPDATE
    int x_val = x;
    int y_val = y;
    int x_centered = x_val - 2048;
    int y_centered = x_val - 2048;
    // Map joystick to motor direction from y-axis
    int motor_a_dir = y_centered >= 0 ? 1 : -1;
    int motor_b_dir = y_centered >= 0 ? 1 : -1;
    int motor_a_speed = abs(y_centered) * 8192 / 2048;
    int motor_b_speed = abs(y_centered) * 8192 / 2048;
    // Add turning effect from x-axis
    motor_a_speed -= x_centered * 8192 / 2048;
    motor_b_speed += x_centered * 8192 / 2048;
    // Clamp speeds
    if (motor_a_speed < 0) motor_a_speed = 0;
    if (motor_b_speed < 0) motor_b_speed = 0;
    if (motor_a_speed > 8192) motor_a_speed = 8192;
    if (motor_b_speed > 8192) motor_b_speed = 8192;
    //set_motor_direction();
    //set_motor_speed();
    // Pass PWM values to the proper DC motors depending on the joystick y-axis position
    // Forward
    /*if (y_val > y_centered) {
        m.motor1_rpm_pcm = motor_a_speed;
        m.motor2_rpm_pcm = motor_b_speed;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    // Reverse
    if (y_val < y_centered) {
        m.motor1_rpm_pcm = motor_a_speed;
        m.motor2_rpm_pcm = motor_b_speed;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }*/


    /*
    // Turn Left
    if (x == 8190 && y == -8190) {
        m.motor1_rpm_pcm = 6172;
        m.motor2_rpm_pcm = 8190;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    else if (x == 8190 && y == 8190) {
        m.motor1_rpm_pcm = 8190;
        m.motor2_rpm_pcm = 6172;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    else if (x == -8190 && y == -8190) {
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = 6172;
        m.motor4_rpm_pcm = 8190;
    }
    else if (x == -8190 && y == 8190) {
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = 8190;
        m.motor4_rpm_pcm = 6172;
    }*/
    // FORWARD AND REVERSE
    //if ((x > 1500) && (y > 700 && y < 850)) {
    
    else if ((x > 1500) && (y > -2500 && y < 2500)) {
        //ESP_LOGW("ESP-NOW", "FORWARD");
        // Both sides rotate in forward direction.
        m.motor1_rpm_pcm = x;   // left, forward
        m.motor2_rpm_pcm = x;   // right, forward
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    //else if ((x < 0) && (y > 700 && y < 850)) {
    else if ((x < 0) && (y > -2500 && y < 2500)) {
        //ESP_LOGW("ESP-NOW", "REVERSE");
        // Both sides rotate in reverse direction.
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = -x;
        m.motor4_rpm_pcm = -x;
    }
    // ROTATE CLOCKWISE AND COUNTER CLOCKWISE
    else if ((x > -2500 && x < 2500) && (y < 0)) {
        //ESP_LOGW("ESP-NOW", "LEFT");
        // Left side rotates in forward direction, right side rotates in reverse direction.
        m.motor1_rpm_pcm = 0;//-y;
        m.motor2_rpm_pcm = -y;//0;
        m.motor3_rpm_pcm = 0;//-y;
        m.motor4_rpm_pcm = -y;//0;
    }
    else if ((x > -2500 && x < 2500) && (y > 900)) {
        //ESP_LOGW("ESP-NOW", "RIGHT");
        // Right side rotates in forward direction, left side rotates in reverse direction.
        m.motor1_rpm_pcm = y;//0;
        m.motor2_rpm_pcm = 0;//y; 
        m.motor3_rpm_pcm = y;//0;
        m.motor4_rpm_pcm = 0;//y; 
    }
    else {
        //ESP_LOGW("ESP-NOW", "STAND STILL");
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, m.motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, m.motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, m.motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, m.motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

}

static void update_motors_pwm (int pwm_motor_1, int pwm_motor_2) {

    

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, m.motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, m.motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, m.motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, m.motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

    ESP_LOGW("MTR LGC", "M1: %d, M2: %d, M3: %d, M4: %d \n ===========================",
        m.motor1_rpm_pcm,
        m.motor2_rpm_pcm,
        m.motor3_rpm_pcm,
        m.motor4_rpm_pcm);
}

/*static void rc_get_raw_data() {

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[0][0]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw[0][1]));
    ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN0, adc_raw[0][0]);
    ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1, adc_raw[0][1]);
    ESP_LOGI("Joystick L/R", "Position: %d (%d)", rescale_raw_val(adc_raw[0][0]),  check_motor_pcm(rescale_raw_val(adc_raw[0][0])));
    ESP_LOGI("Joystick F", "Position: %d (%d)", rescale_raw_val(adc_raw[0][1]), check_motor_pcm(rescale_raw_val(adc_raw[0][1])));
    ESP_LOGW("Joystick", " sample %d, (x,y): (%d, %d)", sample, x, y);

    if (s < sample) {
        x_sum += check_motor_pcm(rescale_raw_val(adc_raw[0][0]));
        y_sum += check_motor_pcm(rescale_raw_val(adc_raw[0][1]));
        s ++;
    }
    else if (s == sample) {
        //x = check_motor_pcm(rescale_raw_val(adc_raw[0][0]));
        //y = check_motor_pcm(rescale_raw_val(adc_raw[0][1]));
        x = x_sum / sample;
        y = y_sum / sample;
    //x = buf.x_axis;
    //y = buf.y_axis;
    

    if ((x > 0 && x < 500) && (y > 500)) {
        ESP_LOGW("RC", "FORWARD");
        // Both sides rotate in forward direction.
        m.motor1_rpm_pcm = y;   // left, forward
        m.motor2_rpm_pcm = y;   // right, forward
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    else if ((x > 0 && x < 500) && (y < -200)) {
        ESP_LOGW("RC", "REVERSE");
        // Both sides rotate in reverse direction.
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = -y;
        m.motor4_rpm_pcm = -y;
    }
    else if ((y < 0 && y > -200) && (x < -1000)) {
        ESP_LOGW("RC", "LEFT");
        // Left side rotates in forward direction, right side rotates in reverse direction.
        m.motor1_rpm_pcm = -x;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = -x;
        m.motor4_rpm_pcm = 0;
    }
    else if ((y < 0 && y > -200) && (x > 1000)) {
        ESP_LOGW("RC", "RIGHT");
        // Right side rotates in forward direction, left side rotates in reverse direction.
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = x; 
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = x; 
    }
    else {
        ESP_LOGW("RC", "STAND STILL");
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    s++;
    }
    else {
        x_sum = 0;
        y_sum = 0;
        s = 0;
    }

    ESP_LOGI("PWM", "Motor 1 PWM: %d", m.motor1_rpm_pcm);
    ESP_LOGI("PWM", "Motor 2 PWM: %d", m.motor2_rpm_pcm);
    ESP_LOGI("PWM", "Motor 3 PWM: %d", m.motor3_rpm_pcm);
    ESP_LOGI("PWM", "Motor 4 PWM: %d", m.motor4_rpm_pcm);

    if (do_calibration1_chan0) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN0, voltage[0][0]);
    }
    if (do_calibration1_chan1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
        ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN1, voltage[0][1]);
    }
}*/

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