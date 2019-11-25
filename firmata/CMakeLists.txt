# EFM32 Emlib CMake file
project(firmata)

# find . -name '*.c' | sort -u
add_library(${PROJECT_NAME}
        src/Firmata.cpp
        src/FirmataMarshaller.cpp
        src/FirmataParser.cpp
       )

target_include_directories(${PROJECT_NAME} PUBLIC 
        inc)

target_link_libraries(${PROJECT_NAME} balena serial)