idf_component_register(SRCS "example.cpp"
                            "MSM_CAN.cpp"
                       PRIV_REQUIRES spi_flash esp_driver_twai esp_driver_gpio esp_timer  
                       INCLUDE_DIRS ".")
