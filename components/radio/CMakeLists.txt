idf_component_register(SRCS "radio.c" 
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio hamming nrf24l01plus 
                    INCLUDE_DIRS "include")