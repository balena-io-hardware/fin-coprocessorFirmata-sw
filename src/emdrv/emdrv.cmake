# EFM32 Emdrv CMake file
#
# Configures the emdrv and adds it to the build

##### Files #####

# Headers
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/dmadrv/inc
    ${CMAKE_CURRENT_LIST_DIR}/dmadrv/config
    ${CMAKE_CURRENT_LIST_DIR}/gpiointerrupt/inc
    ${CMAKE_CURRENT_LIST_DIR}/gpiointerrupt/config
    ${CMAKE_CURRENT_LIST_DIR}/nvm/inc
    ${CMAKE_CURRENT_LIST_DIR}/nvm/config
    ${CMAKE_CURRENT_LIST_DIR}/rtcdrv/inc
    ${CMAKE_CURRENT_LIST_DIR}/rtcdrv/config
    ${CMAKE_CURRENT_LIST_DIR}/sleep/inc
    ${CMAKE_CURRENT_LIST_DIR}/sleep/config
    ${CMAKE_CURRENT_LIST_DIR}/spidrv/inc
    ${CMAKE_CURRENT_LIST_DIR}/spidrv/config
    ${CMAKE_CURRENT_LIST_DIR}/tempdrv/inc
    ${CMAKE_CURRENT_LIST_DIR}/tempdrv/config
    ${CMAKE_CURRENT_LIST_DIR}/uartdrv/inc
    ${CMAKE_CURRENT_LIST_DIR}/uartdrv/config
    ${CMAKE_CURRENT_LIST_DIR}/ustimer/inc
    ${CMAKE_CURRENT_LIST_DIR}/ustimer/config
    ${CMAKE_CURRENT_LIST_DIR}/common/inc
    ${CMAKE_CURRENT_LIST_DIR}/common/config
)

# Source files
FILE(
    GLOB 
    EMDRV_SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/dmadrv/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/ustimer/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/gpiointerrupt/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/nvm/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/nvm/config/*.c
    ${CMAKE_CURRENT_LIST_DIR}/rtcdrv/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/sleep/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/spidrv/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/tempdrv/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/uartdrv/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/ustimer/src/*.c
)

##### Outputs #####

# Create emlib library
add_library(emdrv ${EMDRV_SOURCES})

# Add library to build
set(LIBS ${LIBS} emdrv)