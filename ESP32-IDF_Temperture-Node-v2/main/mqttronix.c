/*  
    MQTT for Electronics

    Author:     Alexander Bobkov
    Date:       October 4, 2025
    Modified:   October 4, 2025

    Built and Compiled with ESP-IDF v5.4.1
    Uses Espressif mqtt component: espressif/mqtt^1.0.0
*/

#include "esp_log.h"
#include "mqtt_client.h"
#include "mqttronix.h"

static float temp_value = 0.0f;
static float pressure_voltage = 0.0f;
static float humidity_value = 0.0f;

static char temp_str[6];

static esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt_publish_task(void *arg) {
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)arg;
    
    while (1) {
        /*/float tsens_value = 0.0f;
        //temperature_sensor_get_celsius(temp_sensor, &tsens_value);
        ESP_LOGW("ESP32-C3", "Temperature value %.02f ℃", temp_value);
        char temp_str[6], battery_voltage_str[8], sys_current_str[8], sys_power_str[8];
        char pwm_1_str[5], pwm_2_str[5], pwm_3_str[5], pwm_4_str[5];
        snprintf(temp_str, sizeof(temp_str), "%.02f", temp_value);
        snprintf(battery_voltage_str, sizeof(battery_voltage_str), "%.02f", battery_voltage);
        snprintf(sys_current_str, sizeof(sys_current_str), "%.02f", sys_current);
        snprintf(sys_power_str, sizeof(sys_power_str), "%.02f", sys_power);
        snprintf(pwm_1_str, sizeof(pwm_1_str), "%d", pwm_1);
        snprintf(pwm_2_str, sizeof(pwm_2_str), "%d", pwm_2);
        snprintf(pwm_3_str, sizeof(pwm_3_str), "%d", pwm_3);
        snprintf(pwm_4_str, sizeof(pwm_4_str), "%d", pwm_4);

        // Publish a message every 5 seconds
        esp_mqtt_client_publish(mqtt_client, "/bitrider/temp", temp_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/battery_voltage", battery_voltage_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/sys_current", sys_current_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/sys_power", sys_power_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/pwm-1", pwm_1_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/pwm-2", pwm_2_str, 0, 1, 0);
        //esp_mqtt_client_publish(mqtt_client, "/bitrider/pwm-3", pwm_3_str, 0, 1, 0);
        //esp_mqtt_client_publish(mqtt_client, "/bitrider/pwm-4", pwm_4_str, 0, 1, 0);*/
        esp_mqtt_client_publish(mqtt_client, "nodes/outdoors/foxie2/sensors/temperature", temp_str, 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(MQTT_TAG, "Called task to publish topic /bitrider/temp");
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI("MQTTronix", "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_publish(client, "nodes/outdoor/foxie2/temperature", "1.0", 0, 1, 0);
            mqtt_client = client;
            xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 8192, NULL, 5, NULL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI("MQTTronix", "MQTT_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
}

void mqttronix_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void mqttronix_update_temp (float temp) { 
    temp_value = temp; 
    snprintf(temp_str, sizeof(temp_str), "%.02f", temp_value);
}