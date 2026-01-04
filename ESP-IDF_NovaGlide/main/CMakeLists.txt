idf_component_register(
    SRCS "ESP-IDF_NovaGlide.c" "system_init.c" "scheduler.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_netif
        nvs_flash
        motors
        adc
        wifi_sys
        mqtt_sys
        espnow_sys
        ina219_sensor
        ultrasonic_sensor)
