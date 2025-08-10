idf_component_register(SRCS "joystick_inputs.c" 
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio mcp3208 esp_timer
                    INCLUDE_DIRS ".")