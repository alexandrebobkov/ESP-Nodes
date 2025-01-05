#ifndef JOYSTICK_H
#define JOYSTICK_H

static unsigned uint8_t x, y;
adc_oneshot_unit_handle_t adc1_x_handle;

static void joystick_get_raw_xy() {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN0, &x));
}

#endif