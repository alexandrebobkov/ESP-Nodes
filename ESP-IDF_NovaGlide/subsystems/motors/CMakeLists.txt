idf_component_register(
    SRCS "motors.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_gpio
)
