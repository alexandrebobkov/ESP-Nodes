idf_component_register(
    SRCS "ina219_sensor.c"
         "temp_sensor.c"
         "ultrasonic_hal.c"
    INCLUDE_DIRS "."
    REQUIRES driver
             esp_adc
             i2c_bus
    PRIV_REQUIRES i2c_bus
                  esp-idf-lib__ina219
                  esp-idf-lib__i2cdev)
