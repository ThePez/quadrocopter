idf_component_register(SRCS "common_functions.c"
                    REQUIRES esp_driver_spi esp_driver_gpio esp_driver_i2c
                    INCLUDE_DIRS ".")
