#include "adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "ADC";
static adc_oneshot_unit_handle_t adc1;

static void adc_read_impl(adc_system_t *self)
{
    int x = 0, y = 0;
    adc_oneshot_read(adc1, ADC_CHANNEL_0, &x);
    adc_oneshot_read(adc1, ADC_CHANNEL_1, &y);
    self->x_raw = x;
    self->y_raw = y;
}

static void adc_update_impl(adc_system_t *self, TickType_t now)
{
    (void)now;
    adc_read_impl(self);
}

void adc_system_init(adc_system_t *sys)
{
    adc_oneshot_unit_init_cfg_t cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&cfg, &adc1);

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

    adc_oneshot_config_channel(adc1, ADC_CHANNEL_0, &ch_cfg);
    adc_oneshot_config_channel(adc1, ADC_CHANNEL_1, &ch_cfg);

    sys->x_raw = 0;
    sys->y_raw = 0;
    sys->read = adc_read_impl;
    sys->update = adc_update_impl;
}
