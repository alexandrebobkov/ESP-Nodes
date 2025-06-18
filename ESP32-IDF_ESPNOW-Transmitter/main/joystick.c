#include "sensors_data.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "config.h"
#include "joystick.h"

#include "sensors_data.h"

extern sensors_data_t buffer;
static int x, y; // Joystick x and y positions
adc_oneshot_unit_handle_t adc_xy_handle;
//uint8_t receiver_mac[ESP_NOW_ETH_ALEN]       = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot
//extern uint8_t receiver_mac[ESP_NOW_ETH_ALEN]; 

uint8_t broadcast_mac[ESP_NOW_ETH_ALEN]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};     // Broadcast MAC address
uint8_t receiver_mac[ESP_NOW_ETH_ALEN]       = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot
uint8_t transmitter_mac[ESP_NOW_ETH_ALEN]    = {0x34, 0xB7, 0xDA, 0xF9, 0x33, 0x8D};     // MAC address of Remote Control


// Function to delete peer (i.e. when communication error occurs)
void deleteDev (void) {
    uint8_t delStatus = esp_now_del_peer(receiver_mac);
    if (delStatus != 0) {
        ESP_LOGE("ESP-NOW", "Could not delete peer");
    }
}
// Function to delete peer (i.e. when communication error occurs)
void deletePeer (void) {
    uint8_t delStatus = esp_now_del_peer(receiver_mac);
    if (delStatus != 0) {
        ESP_LOGE("ESP-NOW", "Could not delete peer");
    }
}

int convert_axis_to_pwm(int axis_value) {
    // Convert the joystick axis value to a PWM value
    // Assuming axis_value is in the range of 0-4095 for a 12-bit ADC
    // and we want to map it to a PWM range of 0-255
    return (axis_value * 255) / 4095;
}

void get_joystick_xy_axis(int *x_axis, int *y_axis) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_0, x_axis));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_1, y_axis));
}

void sendRawData(void) {
    
    buffer.crc = 0;
    buffer.x_axis = 240;
    buffer.y_axis = 256;
    buffer.nav_bttn = 0;
    buffer.motor1_rpm_pcm = 0;
    buffer.motor2_rpm_pcm = 0;
    buffer.motor3_rpm_pcm = 0;
    buffer.motor4_rpm_pcm = 0;

    get_joystick_xy_axis(&x, &y);
    buffer.x_axis = x;
    buffer.y_axis = y;

    // Display brief summary of data being sent.
    ESP_LOGI("ESP-NOW", "Joystick (x,y) position ( %d, %d )", buffer.x_axis, buffer.y_axis);
    ESP_LOGI("ESP-NOW", "pcm 1, pcm 2 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor1_rpm_pcm, (uint8_t)buffer.motor2_rpm_pcm);
    ESP_LOGI("ESP-NOW", "pcm 3, pcm 4 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor3_rpm_pcm, (uint8_t)buffer.motor4_rpm_pcm);

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
        deleteDev();
    }
}