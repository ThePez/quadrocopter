idf_component_register(SRCS "lis3DH.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio
                    INCLUDE_DIRS "include")
