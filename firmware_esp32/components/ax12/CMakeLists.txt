set(DYNAMIXEL2ESPRESSIF_DIR "dynamixel2espressif")

idf_component_register(
    SRCS
        "ax12.cpp"
        "${DYNAMIXEL2ESPRESSIF_DIR}/actuator.cpp"
        "${DYNAMIXEL2ESPRESSIF_DIR}/arduino_droppings.cpp"
        "${DYNAMIXEL2ESPRESSIF_DIR}/dynamixel2espressif.cpp"
        "${DYNAMIXEL2ESPRESSIF_DIR}/master.cpp"
        "${DYNAMIXEL2ESPRESSIF_DIR}/port_handler.cpp"
        "${DYNAMIXEL2ESPRESSIF_DIR}/protocol.cpp"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "${DYNAMIXEL2ESPRESSIF_DIR}/include"
    REQUIRES
        driver
        esp_driver_gpio
        esp_driver_uart
        esp_timer
        log
)
