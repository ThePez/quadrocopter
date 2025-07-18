idf_component_register(SRCS "sensor_fusion.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio sh2 esp_timer
                    INCLUDE_DIRS "include")

