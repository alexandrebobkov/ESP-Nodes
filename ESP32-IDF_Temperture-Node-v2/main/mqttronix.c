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

void mqttronix_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}