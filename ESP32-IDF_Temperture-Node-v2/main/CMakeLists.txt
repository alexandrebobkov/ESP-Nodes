idf_component_register(SRCS "mqttronix.c" "hello_world_main.c"
                    PRIV_REQUIRES spi_flash esp_netifs
                    INCLUDE_DIRS ".")
