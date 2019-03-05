# Base EFM32 CMake file
#
# This can be used as is as a project base, or by adding the efm32-base
# repository as a submodule to another project, copying this CMakeLists file
# to the top level directory, and updating the BASE_LOCATION variable to reflect this
# change
#
## Copyright (c) 2016 Ryan Kurte
# This file is covered under the MIT license available at: https://opensource.org/licenses/MIT

###### Project Environment #####

# Set minimum CMake version
cmake_minimum_required(VERSION 2.8.4)

# Set efm32-base project location
set(BASE_LOCATION .)

# Set the compiler (must be prior to project setup)
include(${BASE_LOCATION}/toolchain/arm-gcc.cmake)

##### Project Setup #####

# Set our output target
set(TARGET firmata)

# Configure project and languages
project(${TARGET} C CXX ASM)

# Set device
if(NOT DEVICE)
set(DEVICE EFM32GG990F1024)
endif()

# Set build
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE DEBUG)
endif()

##### Modules #####

# Libraries can be added to the LIBS variable
# or manually included here.

# Add base libs (emlib, emdrv, CMSIS, device files)
include(${BASE_LOCATION}/toolchain/firmata.cmake)

##### Files #####

# Add project headers
include_directories(
	${BASE_LOCATION}/include/balena
	${BASE_LOCATION}/include/firmata
	${BASE_LOCATION}/include/serial
	${BASE_LOCATION}/include/config
)

# Add project sources
set(SOURCES
	${BASE_LOCATION}/source/StandardFirmata.cpp
	${BASE_LOCATION}/source/protocol/balena/balena.cpp
	${BASE_LOCATION}/source/protocol/firmata/Firmata.cpp
	${BASE_LOCATION}/source/protocol/firmata/FirmataMarshaller.cpp
	${BASE_LOCATION}/source/protocol/firmata/FirmataParser.cpp
	${BASE_LOCATION}/source/protocol/serial/Serial.cpp
)

##### Outputs #####

# Generate executable and link
add_executable(${TARGET} ${SOURCES})
target_link_libraries(${TARGET} ${LIBS})

##### Post build #####

# Add post build commands
include(${BASE_LOCATION}/toolchain/post-build.cmake)

# Add JLink commands
include(${BASE_LOCATION}/toolchain/jlink.cmake)

