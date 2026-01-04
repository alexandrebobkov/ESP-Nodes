idf_component_register(
    SRCS "temp_sensor.c"
        "ina219_sensor.c"
        "ultrasonic_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc i2c
)
