#include "adc.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "ADC";

// ADC channels for joystick
#define ADC_CHANNEL_X ADC_CHANNEL_0  // GPIO 0
#define ADC_CHANNEL_Y ADC_CHANNEL_1  // GPIO 1

static adc_oneshot_unit_handle_t adc_handle = NULL;

static void adc_read_impl(adc_system_t *self, int *x, int *y) {
    int x_val, y_val;

    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_X, &x_val));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_Y, &y_val));

    *x = x_val;
    *y = y_val;

    self->x_raw = x_val;
    self->y_raw = y_val;
}

static void adc_update_impl(adc_system_t *self, TickType_t now) {
    (void)now;
    // ADC reading happens on demand via read() function
}

void adc_system_init(adc_system_t *sys) {
    sys->last_reading = 0;
    sys->x_raw = 0;
    sys->y_raw = 0;
    sys->update = adc_update_impl;
    sys->read = adc_read_impl;

    // Configure ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configure ADC channels
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_X, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_Y, &config));

    ESP_LOGI(TAG, "ADC initialized");
}
