cmake_minimum_required(VERSION 3.30.5)

set(SOURCES
        src/AS5600.cpp
)

set(COMPONENTS
        foc
        i2c
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS include REQUIRES ${COMPONENTS})
