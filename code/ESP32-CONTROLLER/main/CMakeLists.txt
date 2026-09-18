# ESP32-CONTROLLER main component CMakeLists.txt
idf_component_register(
    SRCS 
        "main.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        driver
        esp_wifi
        esp_now
        nvs_flash
        freertos
        esp_adc
        esp_timer
        log
)
