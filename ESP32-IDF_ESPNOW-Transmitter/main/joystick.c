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

void sendData (void)
{
    buffer.crc = 0;
    buffer.x_axis = 240;
    buffer.y_axis = 256;
    buffer.nav_bttn = 0;
    buffer.motor1_rpm_pcm = 0; //10;
    buffer.motor2_rpm_pcm = 0;
    buffer.motor3_rpm_pcm = 0;
    buffer.motor4_rpm_pcm = 0;

    get_joystick_xy(&x, &y);
    //ESP_LOGI("(x, y)", "[ %d, %d ]", x, y);
    buffer.x_axis = x;
    buffer.y_axis = y;

    // Display brief summary of data being sent.
    ESP_LOGI(TAG, "Joystick (x,y) position ( %d, %d )", buffer.x_axis, buffer.y_axis);
    ESP_LOGI(TAG, "pcm 1, pcm 2 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor1_rpm_pcm, (uint8_t)buffer.motor2_rpm_pcm);
    ESP_LOGI(TAG, "pcm 3, pcm 4 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor3_rpm_pcm, (uint8_t)buffer.motor4_rpm_pcm);

    // Call ESP-NOW function to send data (MAC address of receiver, pointer to the memory holding data & data length)
    uint8_t result = esp_now_send(receiver_mac, (uint8_t *)&buffer, sizeof(buffer));

    // If status is NOT OK, display error message and error code (in hexadecimal).
    if (result != 0) {
        ESP_LOGE("sendData()", "Error sending data! Error code: 0x%04X", result);
        ESP_LOGE("sendData()", "esp_now_send() failed: %s", esp_err_to_name(result));
        ESP_LOGE("sendData()", "Ensure that receiver is powered-on.");
        ESP_LOGE("sendData()", "Ensure that received MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
                 receiver_mac[0], receiver_mac[1], receiver_mac[2],
                 receiver_mac[3], receiver_mac[4], receiver_mac[5]);
        deletePeer();
    }
}