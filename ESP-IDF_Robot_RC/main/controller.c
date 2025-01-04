#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "common.h"

static const char *TAG = "RC";

void onDataSent (uint8_t *mac_addr, esp_now_send_status_t status) {
    //status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    ESP_LOGW(TAG, "Packet send status: %i", status);
}
void sensors_data_prepare(espnow_data_packet_t *send_packet) {
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

static void rc_send_data_task2 (void *pvParameter) {

    espnow_data_packet_t *send_packet = (espnow_data_packet_t *)pvParameter;

    while (true) {
        //memcpy(send_packet->dest_mac, receiver_mac, ESP_NOW_ETH_ALEN);
        esp_err_t r = esp_now_send(receiver_mac, send_packet->buffer, sizeof(sensors_data_t));//send_packet->len);
        //esp_now_send(send_packet->dest_mac, send_packet->buffer, send_packet->len);

        if (r != ESP_OK) {
            ESP_LOGE(TAG, "Send error.");
            vTaskDelete(NULL);
            break;
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}