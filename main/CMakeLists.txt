idf_component_register(
    SRCS "hello_world_main.cpp"
    PRIV_REQUIRES spi_flash
    REQUIRES hf-ws2812-rmt-driver
    INCLUDE_DIRS "." "../hf-hal/utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include"
)
