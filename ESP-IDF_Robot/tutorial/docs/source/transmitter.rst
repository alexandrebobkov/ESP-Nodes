TRANSMITTER
===========

Configuration Variables
-----------------------

.. code-block:: c

    uint8_t receiver_mac[ESP_NOW_ETH_ALEN]  = {0xe4, 0xb0, 0x63, 0x17, 0x9e, 0x44};

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

Sending & Ecapsulating Data
----------------------------

.. code-block:: c

    void sendData (void) {

        buffer.x_axis = x_axis;
        buffer.y_axis = y_axis;
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