/*  
    MQTT for Electronics

    Author:     Alexander Bobkov
    Date:       October 4, 2025
    Modified:   November 1, 2025

    Built and Compiled with ESP-IDF v5.4.1
    Uses Espressif mqtt component: espressif/mqtt^1.0.0
*/

#include "esp_log.h"
#include "esp_crc.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "mqtt_client.h"
#include "mqttronix.h"

//static float temp_value = 0.0f;
//static float pressure_value = 0.0f;
//static float humidity_value = 0.0f;

static char temp_str[6];
static char humidity_str[6];
static char pressure_str[6];

static esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt_publish_task(void *arg) {
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)arg;
    
    while (1) {
        esp_mqtt_client_publish(mqtt_client, "nodes/outdoors/foxie1/sensors/temperature", temp_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "nodes/outdoors/foxie1/sensors/humidity", humidity_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "nodes/outdoors/foxie1/sensors/pressure", pressure_str, 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(1500));
        ESP_LOGI(MQTT_TAG, "Called task to publish topic \"nodes/outdoors/foxie1/#\"");
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI("MQTTronix", "MQTT_EVENT_CONNECTED");
            //esp_mqtt_client_publish(client, "nodes/outdoor/foxie2/temperature", "1.0", 0, 1, 0);
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

/* ESP-NOW */
// Wi-Fi should start before using ESP-NOW
void wifi_init()
{
    /*
    * STAND-ALONE
    */
    /*ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));//ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    #endif*/

    /*
    * WI-FI
    */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));//ESPNOW_WIFI_MODE));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,//"IoT_bots2",
            .password = WIFI_PASSWORD,// "208208208",
        },
    };
    ESP_ERROR_CHECK (esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_start());
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    ESP_ERROR_CHECK( esp_wifi_connect() );
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
    //temp_value = temp; 
    if (temp >= -35.0f && temp <= 50.0f) {
        snprintf(temp_str, sizeof(temp_str), "%.1f", temp);
    }
}
void mqttronix_update_humidity (float humidity) { 
    //humidity_value = humidity;
    if (humidity >= 0.0f && humidity <= 100.0f) {
        snprintf(humidity_str, sizeof(humidity_str), "%.0f", humidity);
    }
}
void mqttronix_update_pressure (float pressure) { 
    //pressure_value = pressure;
    snprintf(pressure_str, sizeof(pressure_str), "%.0f", pressure);
}