idf_component_register(SRCS "i2c_io.c" "i2c_multidevice.c"
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver
                    INCLUDE_DIRS ".")
