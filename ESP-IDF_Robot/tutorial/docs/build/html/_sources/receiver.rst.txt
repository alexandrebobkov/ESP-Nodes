RECEIVER
========

Configuration Variables
-----------------------

.. code-block:: c

    uint8_t transmitter_mac[ESP_NOW_ETH_ALEN] = {0x9C, 0x9E, 0x6E, 0x14, 0xB5, 0x54};

    typedef struct {
        int         x_axis;             // Joystick x-position
        int         y_axis;             // Joystick y-position
        bool        nav_bttn;           // Joystick push button
        bool        led;                // LED ON/OFF state
        uint8_t     motor1_rpm_pwm;     // PWMs for 4 DC motors
        uint8_t     motor2_rpm_pwm;
        uint8_t     motor3_rpm_pwm;
        uint8_t     motor4_rpm_pwm;
    } __attribute__((packed)) sensors_data_t;

Receiving & De-Ecapsulating Data
--------------------------------

.. code-block:: c

    void onDataReceived (const uint8_t *mac_addr, const uint8_t *data, uint8_t data_len) {

        ... ... ...
        ... ... ...

        ESP_LOGI(TAG, "Data received from: %02x:%02x:%02x:%02x:%02x:%02x, len=%d", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], data_len);
        memcpy(&buf, data, sizeof(buf));

        x_axis = buf.x_axis;
        y_axis = buf.y_axis;

        ... ... ...
        ... ... ...
    }

Main Function
-------------

.. code-block:: c

    #include <string.h>
    #include "freertos/FreeRTOS.h"
    #include "nvs_flash.h"
    #include "esp_err.h"

    ... ... ...
    ... ... ...

    void app_main(void) {

        ... ... ...
        ... ... ...

        // Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK( nvs_flash_erase() );
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK( ret );
        wifi_init();
        ESP_ERROR_CHECK(esp_now_init());

        esp_now_peer_info_t transmitterInfo = {0};
        memcpy(transmitterInfo.peer_addr, transmitter_mac, ESP_NOW_ETH_ALEN);
        transmitterInfo.channel = 0; // Current WiFi channel
        transmitterInfo.ifidx = ESP_IF_WIFI_STA;
        transmitterInfo.encrypt = false;
        ESP_ERROR_CHECK(esp_now_add_peer(&transmitterInfo));

        ESP_ERROR_CHECK(esp_now_register_recv_cb((void*)onDataReceived));

        system_led_init();

        ... ... ...
        ... ... ...
    }