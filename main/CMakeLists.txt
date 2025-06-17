idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES spi_flash hf-hal hf-ws2812-rmt-driver threads
                       INCLUDE_DIRS ".")
