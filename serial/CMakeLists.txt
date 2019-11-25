# EFM32 Emlib CMake file
project(serial)

# find . -name '*.c' | sort -u
add_library(${PROJECT_NAME}
        src/Serial.cpp
       )

target_include_directories(${PROJECT_NAME} PUBLIC 
        inc)

target_link_libraries(${PROJECT_NAME} balena)