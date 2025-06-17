idf_component_register(SRCS "ws2812_test/WS2812TestThread.cpp"
                            "canopen_bldc/CANOpenBLDCThread.cpp"
                            "canopen_bldc/EnhancedCANOpenUtils.cpp"
                       INCLUDE_DIRS "."
                                   "ws2812_test" 
                                   "canopen_bldc"
                       REQUIRES freertos
                               esp_timer
                               esp_common
                               hf-hal
                               hf-ws2812-rmt-driver)
