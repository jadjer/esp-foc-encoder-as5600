cmake_minimum_required(VERSION 3.17)

set(SOURCES
        src/AS5600.cpp
)

set(COMPONENTS
        esp_foc
        esp_i2c
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS include REQUIRES ${COMPONENTS})
