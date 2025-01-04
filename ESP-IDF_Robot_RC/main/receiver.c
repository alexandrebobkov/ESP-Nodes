#include "freertos/FreeRTOS.h"
#include "common.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "controls.h"

void onDataReceived (uint8_t *mac_addr, uint8_t *data, uint8_t data_len) {

    //memcpy(buf, data, data_len);
    /*buf = (sensors_data_t*)data;
    ESP_LOGW(TAG, "Data was received");
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->x_axis);
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->y_axis);
    ESP_LOGI(TAG, "PCM 1: 0x%04x", buf->motor1_rpm_pcm);*/
}

static void rc_task (void *arg) {
    while (true) {
        rc_get_raw_data();

        ESP_LOGI("PWM", "Motor 1 PWM: %d", m.motor1_rpm_pcm);
        ESP_LOGI("PWM", "Motor 2 PWM: %d", m.motor2_rpm_pcm);
        ESP_LOGI("PWM", "Motor 3 PWM: %d", m.motor3_rpm_pcm);
        ESP_LOGI("PWM", "Motor 4 PWM: %d", m.motor4_rpm_pcm);

        //vTaskDelay (10 / portTICK_PERIOD_MS);  // Determines responsiveness  
        vTaskDelay (1000 / portTICK_PERIOD_MS); 
    }
}