idf_component_register(SRCS "bno085_task.cpp"
                    REQUIRES esp32_BNO08x-no_sh2_hal esp_timer
                    INCLUDE_DIRS "include")