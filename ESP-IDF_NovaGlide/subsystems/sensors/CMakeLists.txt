idf_component_register(
    SRCS "ina219_sensor.c"
         "temp_sensor.c"
         "ultrasonic_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc esp_driver_i2c
    REQUIRES driver
             esp_adc
             esp-idf-lib__ina219
             esp-idf-lib__ultrasonic
             esp-idf-lib__i2cdev)
