idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp32_i2cbus esp_system
                    )
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)