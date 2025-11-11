idf_component_register(
    SRCS "src/I2Cbus.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_gpio log
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
