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

pico_generate_pio_header(z80d ${CMAKE_CURRENT_LIST_DIR}/Z80Bus.pio)

target_sources(z80d PRIVATE
    main.cpp
    Z80Bus.cpp
    mri_platform.cpp
    )

target_include_directories(z80d PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

# mri submodule files.
target_sources(z80d PRIVATE
    mri/core/buffer.c
    mri/core/cmd_break_watch.c
    mri/core/cmd_common.c
    mri/core/cmd_continue.c
    mri/core/cmd_file.c
    mri/core/cmd_memory.c
    mri/core/cmd_query.c
    mri/core/cmd_registers.c
    mri/core/cmd_step.c
    mri/core/cmd_thread.c
    mri/core/cmd_vcont.c
    mri/core/context.c
    mri/core/gdb_console.c
    mri/core/libc.c
    mri/core/memory.c
    mri/core/mri.c
    mri/core/packet.c
    mri/core/token.c
    mri/core/try_catch.c
    )

target_include_directories(z80d PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/mri
    )

target_compile_definitions(z80d PRIVATE
    MRI_ALWAYS_USE_HARDWARE_BREAKPOINT=1
    MRI_UINT_TYPE=uint16_t
    MRI_INT_TYPE=int16_t
)

target_link_libraries(z80d PRIVATE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_pwm
    )
pico_add_extra_outputs(z80d)

pico_enable_stdio_usb(z80d 1)
pico_enable_stdio_uart(z80d 0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)

add_custom_target(size_elf ALL
        COMMAND arm-none-eabi-size $<TARGET_FILE:${PROJECT_NAME}>
        COMMENT "Sizing  ${PROJECT_NAME}"
        VERBATIM
)
