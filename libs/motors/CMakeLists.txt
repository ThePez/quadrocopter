idf_component_register(SRCS "motors.c"
                    PRIV_REQUIRES esp_driver_mcpwm common_functions
                    INCLUDE_DIRS ".")
