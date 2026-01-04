idf_component_register(
    SRCS "ui.c" "dashboard.c"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_driver_gpio)
