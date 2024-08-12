/* Switch demo implementation using button and RGB LED
   
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>


#include <iot_button.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 
#include <esp_log.h>

#include "driver/adc.h"
#include "driver/temperature_sensor.h"
#include "esp_adc_cal.h"

#include <app_reset.h>
#include <ws2812_led.h>
#include "app_priv.h"

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0

/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    CONFIG_EXAMPLE_OUTPUT_GPIO

#define SYS_LED_1           CONFIG_SWITCH_LED_1
#define SYS_LED_2           CONFIG_SWITCH_LED_2
#define LIGHT_SENSOR        CONFIG_LIGHT_SENSOR

static bool g_power_state = DEFAULT_POWER;

/* These values correspoind to H,S,V = 120,100,10 */
#define DEFAULT_RED     0
#define DEFAULT_GREEN   25
#define DEFAULT_BLUE    0

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

// Define the name of app for logs.
static const char *TAG = "ESP32-Nodes Rainmaker Switch";
static float a_light;
static float tsens_value;
static int a_light_raw;
static TimerHandle_t sensor_timer;
static temperature_sensor_handle_t temp_sensor = NULL;
static temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
esp_adc_cal_characteristics_t adc1_chars;

/*
* AMBIENT LIGHT SENSOR FUNCTIONS

static void light_sensor_init(void) {
    ESP_LOGI(TAG, "Initializing sensor ...");
}*/

static void light_sensor_update(TimerHandle_t handle) {
    static float delta = 0.25;
    a_light += delta;

    // Obtain raw ADC value from the ambient light sensor connected to GPIO 1
    a_light_raw = adc1_get_raw(ADC1_CHANNEL_1);

    // switch_device
    esp_rmaker_param_update_and_report(
        esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
        esp_rmaker_float((float)a_light_raw));
    
    ESP_LOGI(TAG, "\nAmbient light sensor value: %i", a_light_raw);    
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
    ESP_LOGI(TAG, "\nESP32-C3 Module temperature: %0.2f", tsens_value);
}
void app_sensor_init(void) {
//esp_err_t app_sensor_init(void) {
    a_light = 15.0;
    a_light_raw = 0;
    sensor_timer = xTimerCreate("ambient_light_sensor_update_timer", (REPORTING_PERIOD*250) / portTICK_PERIOD_MS,
        pdTRUE, NULL, light_sensor_update);
    
    if (sensor_timer) {
        xTimerStart(sensor_timer, 0);
        //return ESP_OK;
    }
    //return ESP_FAIL;
}

float app_get_current_temperature() {
    //return a_light;
    return a_light_raw;
}

//static void app_bme280_init() {}

static void app_indicator_set(bool state)
{
    if (state) {
        ws2812_led_set_rgb(DEFAULT_RED, DEFAULT_GREEN, DEFAULT_BLUE);
    } else {
        ws2812_led_clear();
    }
}

static void app_indicator_init(void)
{
    ws2812_led_init();
    app_indicator_set(g_power_state);
}
static void push_btn_cb(void *arg)
{
    bool new_state = !g_power_state;
    app_driver_set_state(new_state);
#ifdef CONFIG_EXAMPLE_ENABLE_TEST_NOTIFICATIONS
    /* This snippet has been added just to demonstrate how the APIs esp_rmaker_param_update_and_notify()
     * and esp_rmaker_raise_alert() can be used to trigger push notifications on the phone apps.
     * Normally, there should not be a need to use these APIs for such simple operations. Please check
     * API documentation for details.
     */
    if (new_state) {
        esp_rmaker_param_update_and_notify(
                esp_rmaker_device_get_param_by_name(switch_device, ESP_RMAKER_DEF_POWER_NAME),
                esp_rmaker_bool(new_state));
    } else {
        esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_name(switch_device, ESP_RMAKER_DEF_POWER_NAME),
                esp_rmaker_bool(new_state));
        esp_rmaker_raise_alert("Switch was turned off");
    }
#else
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(switch_device, ESP_RMAKER_DEF_POWER_NAME),
            esp_rmaker_bool(new_state));
#endif
}

static void set_power_state(bool target)
{
    //gpio_set_level(OUTPUT_GPIO, target);
    gpio_set_level(SYS_LED_1, target);
    gpio_set_level(SYS_LED_2, !target);
    app_indicator_set(target);
}

void app_driver_init()
{
    button_handle_t btn_handle = iot_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        /* Register a callback for a button tap (short press) event */
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_TAP, push_btn_cb, NULL);
        /* Register Wi-Fi reset and factory reset functionality on same button */
        app_reset_button_register(btn_handle, WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
    }

    /* Configure power */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    //io_conf.pin_bit_mask = ((uint64_t)1 << SYS_LED_1);//OUTPUT_GPIO);
    io_conf.pin_bit_mask = (((uint64_t)1 << SYS_LED_1) | ((uint64_t)1 << SYS_LED_2));
    /* Configure the GPIO */
    gpio_config(&io_conf);
    app_indicator_init();

    /* Configure ambient light sensor ADC GPIO */
    // Set the attenuation parameter of ADC; GPIO 1 to 12db
    adc1_config_channel_atten(ADC_UNIT_2, ADC_ATTEN_DB_12);
    // Calibrate ADC
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    // Set ADC bit width (resolution)
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);

    /*gpio_config_t sensor_io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
    };
    sensor_io_conf.pin_bit_mask = (uint64_t)1 << LIGHT_SENSOR;
    gpio_config(&sensor_io_conf);*/
    
    app_sensor_init();

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
}

int IRAM_ATTR app_driver_set_state(bool state)
{
    if(g_power_state != state) {
        g_power_state = state;
        set_power_state(g_power_state);
    }
    return ESP_OK;
}

bool app_driver_get_state(void)
{
    return g_power_state;
}
