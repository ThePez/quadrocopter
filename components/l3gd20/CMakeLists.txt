idf_component_register(SRCS "l3gd20.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio
                    INCLUDE_DIRS "include")
