#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

{
    //sensors_data_t *buffer;
    //malloc(sizeof(sensors_data_t));
    //send_packet->buffer = &buffer;
    //sensors_data_t *buffer = (sensors_data_t *)send_packet->buffer;
    sensors_data_t *buffer = (sensors_data_t *)send_packet->buffer;
    assert(send_packet->len >= sizeof(sensors_data_t));

    buffer->type = 1;
    buffer->crc = 0;
    buffer->x_axis = 0;
    buffer->y_axis = 0;
    buffer->nav_bttn = 0;
    buffer->motor1_rpm_pcm = 0;
    buffer->motor2_rpm_pcm = 0;
    buffer->motor3_rpm_pcm = 0;
    buffer->motor4_rpm_pcm = 0;
    ESP_LOGW(TAG, "x-axis: %x", (uint8_t)buffer->x_axis);
    buffer->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buffer, send_packet->len);
}