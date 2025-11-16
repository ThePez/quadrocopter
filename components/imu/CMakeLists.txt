idf_component_register(SRCS "imu.cpp"
                    REQUIRES esp32_BNO08x esp_timer
                    INCLUDE_DIRS ".")