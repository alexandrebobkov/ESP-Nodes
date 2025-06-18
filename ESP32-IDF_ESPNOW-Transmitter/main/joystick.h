#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

//#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sensors_data.h"

/*#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_event.h"*/

#include "config.h"

//static int x, y; // Joystick x and y positions
//static adc_oneshot_unit_handle_t adc_xy_handle;
// Struct holding sensors values
/*typedef struct {
    uint16_t    crc;                // CRC16 value of ESPNOW data
    int         x_axis;             // Joystick x-position
    int         y_axis;             // Joystick y-position
    bool        nav_bttn;           // Joystick push button
    uint8_t     motor1_rpm_pcm;     // PWMs for 4 DC motors
    uint8_t     motor2_rpm_pcm;
    uint8_t     motor3_rpm_pcm;
    uint8_t     motor4_rpm_pcm;
} __attribute__((packed)) sensors_data_t;*/

static int x, y; // Joystick x and y positions
adc_oneshot_unit_handle_t adc_xy_handle;
//extern sensors_data_t buffer;

int convert_axis_to_pwm(int axis_value);
void get_joystick_xy_axis(int *x_axis, int *y_axis);
void sendRawData(void);
void deleteDev(void);

//static sensors_data_t buffer;
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

void joystick_task(void *arg) {
    while (true) {
        joystick_show_raw_xy();
        vTaskDelay (1000 / portTICK_PERIOD_MS);
    }
}

// Function to delete peer (i.e. when communication error occurs)
void deletePeer (void) {
    uint8_t delStatus = esp_now_del_peer(receiver_mac);
    if (delStatus != 0) {
        ESP_LOGE("ESP-NOW", "Could not delete peer");
    }
}
void statusDataSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Data sent successfully to: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGE(TAG, "Error sending data to: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        ESP_LOGE("sendData()", "Error sending data. Error code: 0x%04X", status);
        ESP_LOGE("sendData()", "esp_now_send() failed: %s", esp_err_to_name(status));
        ESP_LOGE("sendData()", "Ensure that receiver is powered-on and MAC is correct.");
        deletePeer();
    }
}
// Function for sending the data to the receiver
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
/*void rc_send_data_task()
{
    while (true) {
        if (esp_now_is_peer_exist(receiver_mac))
            sendData();
        vTaskDelay (250 / portTICK_PERIOD_MS);
    }
}*/

#endif