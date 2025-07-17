idf_component_register(SRCS "sh2.c" "euler.c" "sh2_SensorValue.c" "sh2_util.c" "shtp.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio
                    INCLUDE_DIRS "include")

