idf_component_register(SRCS "bno08x.c"
                            "sh2/sh2.c"
                            "sh2/sh2_SensorValue.c"
                            "sh2/sh2_util.c"
                            "sh2/shtp.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_timer
                    INCLUDE_DIRS "." "sh2")
