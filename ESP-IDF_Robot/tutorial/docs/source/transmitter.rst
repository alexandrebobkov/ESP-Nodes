TRANSMITTER
===========

Configuration Variables
-----------------------

.. code-block:: c

    uint8_t receiver_mac[ESP_NOW_ETH_ALEN]  = {0xe4, 0xb0, 0x63, 0x17, 0x9e, 0x44};

Sending & Ecapsulating Data
----------------------------

.. code-block:: c

    void onDataReceived (const uint8_t *mac_addr, const uint8_t *data, uint8_t data_len) {
        ESP_LOGI(TAG, "Data received from: %02x:%02x:%02x:%02x:%02x:%02x, len=%d", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], data_len);
        memcpy(&buf, data, sizeof(buf));

        x_axis = buf.x_axis;
        y_axis = buf.y_axis
    }

Main Function
-------------

.. code-block:: c

    #include "freertos/FreeRTOS.h"
    #include "nvs_flash.h"
    #include "esp_err.h"

    void app_main(void) {
        // Initialize internal temperature sensor
        chip_sensor_init();

        // Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK( nvs_flash_erase() );
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK( ret );
        wifi_init();
        joystick_adc_init();
        transmission_init();
        system_led_init();
    }