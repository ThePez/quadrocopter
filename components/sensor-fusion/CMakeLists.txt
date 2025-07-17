idf_component_register(SRCS "sensor-fusion.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio sh2
                    INCLUDE_DIRS "include")

