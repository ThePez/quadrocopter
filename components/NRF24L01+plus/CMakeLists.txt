idf_component_register(SRCS "nrf24l01plus.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio
                    INCLUDE_DIRS "include")
