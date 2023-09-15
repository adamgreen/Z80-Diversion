cmake_minimum_required(VERSION 3.26)

if (PICO_SDK_PATH_OVERRIDE)
    set(PICO_SDK_PATH ${PICO_SDK_PATH_OVERRIDE})
else()
    file(REAL_PATH pico-sdk PICO_SDK_PATH BASE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")
endif()

# Pull in SDK (must be before project)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(z80d C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(PICO_BOARD pico)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall -Wextra -Werror
        #-Wno-unused-function # we have some for the docs that aren't called
        -Wno-unused-parameter
        )

# UNDONE: I want asserts enabled in Release builds for early testing.
string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-DNDEBUG" "" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")

# I prefer -g3 for better symbols and -O3 for better optimization.
string(REPLACE "-g" "-g3" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-g" "-g3" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-O2" "-O3" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-O2" "-O3" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")

add_executable(z80d)

pico_generate_pio_header(z80d ${CMAKE_CURRENT_LIST_DIR}/z80_bus.pio)

target_sources(z80d PRIVATE
    main.cpp
    )

target_include_directories(z80d PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_link_libraries(z80d PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
    )
pico_add_extra_outputs(z80d)

pico_enable_stdio_usb(z80d 1)
pico_enable_stdio_uart(z80d 0)

add_custom_target(size_elf ALL
        COMMAND arm-none-eabi-size $<TARGET_FILE:${PROJECT_NAME}>
        COMMENT "Sizing  ${PROJECT_NAME}"
        VERBATIM
)
