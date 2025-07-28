#include "mqtt_client.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"

#include "mqtt.h"

//static const char* MQTT_BROKER_URI = "mqtt://10.100.50.16:1883";//74.14.210.168";//"mqtt://mqtt.techquadbit.net";
static const char* MQTT_BROKER_URI = "mqtt://mqtt.techquadbit.net";//74.14.210.168";//"mqtt://mqtt.techquadbit.net";

static const char* MQTT_TAG = "MQTT_Robot";
static esp_mqtt_client_handle_t mqtt_client = NULL;

extern float tsens_value;

static void mqtt_publish_task(void *arg) {
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)arg;
    
    while (1) {
        //float tsens_value = 0.0f;
        //temperature_sensor_get_celsius(temp_sensor, &tsens_value);
        ESP_LOGW("ESP32-C3", "Temperature value %.02f ℃", tsens_value);
        char temp_str[6];
        snprintf(temp_str, sizeof(temp_str), "%.02f", tsens_value);

        // Publish a message every 5 seconds
        esp_mqtt_client_publish(mqtt_client, "/bitrider/temp", temp_str, 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(MQTT_TAG, "Called task to publish topic /bitrider/temp");
    }
}

void mqtt_publish_temp (float temp) {
    //esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)arg;
    char temp_str[6];
    snprintf(temp_str, sizeof(temp_str), "%.02f", temp);
    esp_mqtt_client_publish(mqtt_client, "/bitrider/temp", temp_str, 0, 1, 0);_
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_publish(client, "/esp/test", "Hello from Alex!", 0, 1, 0);
            mqtt_client = client;
            xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 2048, NULL, 5, NULL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
}

void mqtt_publish() {
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(NULL);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    esp_mqtt_client_start(client);
    
    // Publish a message
    esp_mqtt_client_publish(client, "/esp/test", "Hello from Alex!", 0, 1, 0);
    
    // Stop the client
    esp_mqtt_client_stop(client);
}

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

/*void sta_wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}*/
