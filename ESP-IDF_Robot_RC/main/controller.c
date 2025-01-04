#include <assert.h>
#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "freertos/FreeRTOS.h"

#include "common.h"
#include "config.h"

static const char *TAG = "RC";
static uint8_t flagToSend = 0;

void deletePeer (void) {
    uint8_t delStatus = esp_now_del_peer(receiver_mac);
    if (delStatus != 0) {
        ESP_LOGE("ESP-NOW", "Could not delete peer");
    }
}

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

void sendData (void) {
    // Send data, specify receiver MAC address, pointer to the data being sent, and length of data being sent.
    sensors_data_t buffer;
    buffer.type = 1;
    buffer.crc = 0;
    buffer.x_axis = 240;
    buffer.y_axis = 256;
    buffer.nav_bttn = 0;
    buffer.motor1_rpm_pcm = 10;
    buffer.motor2_rpm_pcm = 0;
    buffer.motor3_rpm_pcm = 0;
    buffer.motor4_rpm_pcm = 0;
    ESP_LOGI(TAG, "x-axis: 0x%04X", (uint8_t)buffer.x_axis);
    ESP_LOGI(TAG, "y-axis: 0x%04X", (uint8_t)buffer.y_axis);
    ESP_LOGI(TAG, "pcm 1: 0x%04X", buffer.motor1_rpm_pcm);
    ESP_LOGI(TAG, "pcm 2: 0x%04X", (uint8_t)buffer.motor2_rpm_pcm);
    ESP_LOGI(TAG, "pcm 3: 0x%04X", (uint8_t)buffer.motor3_rpm_pcm);
    ESP_LOGI(TAG, "pcm 4: 0x%04X", (uint8_t)buffer.motor4_rpm_pcm);

    //uint8_t result = esp_now_send(receiver_mac, &flagToSend, sizeof(flagToSend));
    uint8_t result = esp_now_send(receiver_mac, &buffer, sizeof(buffer));
    //uint8_t result = esp_now_send(receiver_mac, (sensors_data_t *)&buffer, sizeof(buffer));
    if (result != 0) {
        ESP_LOGE("ESP-NOW", "Error sending data!");
        deletePeer();
    }
    else
        ESP_LOGW("ESP-NOW", "Data was sent.");
}
static esp_err_t rc_espnow_init (void) {

    espnow_data_packet_t *send_packet;

    send_packet = malloc(sizeof(espnow_data_packet_t));
    if (send_packet == NULL) {
        ESP_LOGE(TAG, "malloc fail.");
        return ESP_FAIL;
    }

    memset(send_packet, 0, sizeof(espnow_data_packet_t));
    memcpy(send_packet->dest_mac, receiver_mac, ESP_NOW_ETH_ALEN);
    send_packet->len = CONFIG_ESPNOW_SEND_LEN; // 128
    send_packet->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    sensors_data_prepare(send_packet);
    xTaskCreate(rc_send_data_task2, "controller data packets task", 2048, send_packet, 8, NULL);

    return ESP_OK;
}

static void rc_send_data_task (void *arg) {

    while (true) {
        flagToSend = !flagToSend;
        if (esp_now_is_peer_exist(receiver_mac)) {
            sendData();
        }
        vTaskDelay (1000 / portTICK_PERIOD_MS);
    }
}


/*
    ESP-NOW
*/
/* Prepare ESPNOW data to be sent. */
/*void sensors_data_prepare(espnow_data_packet_t *send_packet)
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
}*/
