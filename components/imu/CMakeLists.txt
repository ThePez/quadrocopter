idf_component_register(SRCS "imu.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_driver_i2c lis3dh l3gd20 hmc5883l
                    INCLUDE_DIRS "include")