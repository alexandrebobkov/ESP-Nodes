/*  
    MQTT for Electronics

    Author:     Alexander Bobkov
    Date:       October 4, 2025
    Modified:   October 4, 2025

    Built and Compiled with ESP-IDF v5.4.1
    Uses Espressif mqtt component: espressif/mqtt^1.0.0
*/

#include "mqtt_client.h"
#include "mqttronix.h"

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_publish(client, "nodes/outdoor/foxie2/temperature", "1.0", 0, 1, 0);
            mqtt_client = client;
            xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 8192, NULL, 5, NULL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
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