#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_now.h"

#include "sensors_data.h"
#include "config.h"

esp_now_peer_info_t devices;
static adc_oneshot_unit_handle_t adc_xy_handle;
sensors_data_t buffer;
static int x, y; // Joystick x- and y- axis positions
static int espnow_channel = 1;
void transmission_init();

esp_err_t joystick_adc_init(void) 
{
    adc_oneshot_unit_init_cfg_t adc_init_config_xy = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config_xy, &adc_xy_handle));

    adc_oneshot_chan_cfg_t config_x = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        //.atten = ADC_ATTEN_DB_0,      //  800mV
        //.atten = ADC_ATTEN_DB_2_5,    //  1.1V
        //.atten = ADC_ATTEN_DB_6,      //  1.3V
        //.atten = ADC_ATTEN_DB_12,     //  2.6V
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_chan_cfg_t config_y = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_xy_handle, ADC_CHANNEL_0, &config_x));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_xy_handle, ADC_CHANNEL_1, &config_y));

    return ESP_OK;
}

void joystick_show_raw_xy()
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_0, &x));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_1, &y));
    ESP_LOGI("(x,y)", "( %d, %d )", x, y);
}

static void get_joystick_xy(int *x_axis, int *y_axis)
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_0, x_axis));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_xy_handle, ADC_CHANNEL_1, y_axis));
}

// Function to delete peer (i.e. when communication error occurs)
static void deletePeer (void) 
{
    uint8_t delStatus = esp_now_del_peer(receiver_mac);
    if (delStatus != 0) {
        ESP_LOGE(TAG, "Could not delete peer");
    }
}
static void sendData (void)
{
    buffer.crc = 0;
    buffer.x_axis = 240;
    buffer.y_axis = 256;
    buffer.nav_bttn = 0;
    buffer.led = 0;
    buffer.motor1_rpm_pwm = 0;
    buffer.motor2_rpm_pwm = 0;
    buffer.motor3_rpm_pwm = 0;
    buffer.motor4_rpm_pwm = 0;

    //joystick_show_raw_xy();
    //get_joystick_xy(&x, &y);
    get_joystick_xy(&y, &x);
    //ESP_LOGI("(x, y)", "[ %d, %d ]", x, y);
    buffer.x_axis = x;
    buffer.y_axis = y;

    // Display brief summary of data being sent.
    ESP_LOGI(TAG, "Joystick (x,y) position ( %d, %d )", buffer.x_axis, buffer.y_axis);
    ESP_LOGI(TAG, "pwm 1, pwm 2 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor1_rpm_pwm, (uint8_t)buffer.motor2_rpm_pwm);
    ESP_LOGI(TAG, "pwm 3, pwm 4 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor3_rpm_pwm, (uint8_t)buffer.motor4_rpm_pwm);

    //ESP_LOGI(TAG, "ESP-NOW Channel: %d", devices.channel);
    //ESP_LOGI(TAG, "Wi-Fi Channel: %d", );
    uint8_t channel;
    esp_wifi_get_channel(&channel, NULL);
    ESP_LOGE(TAG, "ESP-NOW Channel: %d", channel);
    // Call ESP-NOW function to send data (MAC address of receiver, pointer to the memory holding data & data length)
    uint8_t result = esp_now_send((uint8_t*)receiver_mac, (uint8_t *)&buffer, sizeof(buffer));
    ESP_LOGI(TAG, "Channel is set at %d", espnow_channel);

    // If status is NOT OK, display error message and error code (in hexadecimal).
    if (result != 0) {
        ESP_LOGE(TAG, "Error sending data! Error code: 0x%04X", result);
        ESP_LOGE(TAG, "esp_now_send() failed: %s", esp_err_to_name(result));
        ESP_LOGE(TAG, "==========================");
        ESP_LOGE(TAG, "Ensure that received MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
                 receiver_mac[0], receiver_mac[1], receiver_mac[2],
                 receiver_mac[3], receiver_mac[4], receiver_mac[5]);
        
        
        deletePeer();
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (espnow_channel < 11) {
            espnow_channel++;
        } else {
            espnow_channel = 1;
        }
        ESP_LOGI(TAG, "Channel is set at %d", espnow_channel);
        transmission_init();
    }
}

// Callback function to handle the status of data transmission
// This function is called when the data is sent or if there is an error.
static void statusDataSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Data sent successfully to: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else if (status == ESP_NOW_SEND_FAIL) {
        ESP_LOGE(TAG, "Error sending data to: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        ESP_LOGE(TAG, "Error sending data. Error code: 0x%04X", status);
        ESP_LOGE(TAG, "esp_now_send() failed: %s", esp_err_to_name(status));
        ESP_LOGE(TAG, "Ensure that receiver is powered-on and MAC is correct.");
        //deletePeer();
        
        //esp_restart();
        if (espnow_channel < 11) {
            espnow_channel++;
        } else {
            espnow_channel = 1;
        }

        esp_now_deinit();  // Stop ESP-NOW
        esp_wifi_set_channel(espnow_channel, WIFI_SECOND_CHAN_NONE);  // Change channel
        esp_now_init();

    }

    vTaskDelay(pdMS_TO_TICKS(5000));
}

/* WiFi should start before using ESPNOW */
void wifi_init()
{
    /*
    * STAND-ALONE
    */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));//ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK( esp_wifi_start());
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE));
    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    #endif
    

    /*
    * WI-FI
    */
    /*
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));//ESPNOW_WIFI_MODE));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "IoT_bots2",
            .password = "208208208",
        },
    };
    ESP_ERROR_CHECK (esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_start());
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_connect() );
    */
}

// Task to periodically send ESPNOW remote control data
static void rc_send_data_task()
{
    while (true) {
        if (esp_now_is_peer_exist((uint8_t*)receiver_mac)) {
            sendData();
        }
        vTaskDelay (1000 / portTICK_PERIOD_MS);
        /*if (esp_now_is_peer_exist((uint8_t*)receiver_2_mac)) {
            sendData();
        }
        vTaskDelay (10 / portTICK_PERIOD_MS);*/
    }
}

void transmission_init()
{
    esp_err_t espnow_ret = esp_now_init();
    if (espnow_ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init() failed: %s", esp_err_to_name(espnow_ret));
        return;
    }
    ESP_LOGI(TAG, "ESPNOW initialized successfully");
    esp_now_register_send_cb(statusDataSend);

    // Set ESP-NOW receiver device configuration values
    memcpy(devices.peer_addr, receiver_mac, 6);
    devices.channel = espnow_channel;
    devices.encrypt = false;
    esp_now_add_peer(&devices);

    //memcpy(devices.peer_addr, receiver_2_mac, 6);
    //esp_now_add_peer(&devices);

    // Defince a task for periodically sending ESPNOW remote control data
    xTaskCreate(rc_send_data_task, "RC", 2048, NULL, 4, NULL);
}