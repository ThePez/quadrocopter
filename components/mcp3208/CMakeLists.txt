idf_component_register(SRCS "mcp3208.c"
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio common_functions
                    INCLUDE_DIRS ".")
