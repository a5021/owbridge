# =============================================================================
# owbridge - a USART/SPI/soft-I2C to 1-Wire bridge on the STM32F030F4P6
# =============================================================================
#
# Dependencies are fetched from GitHub at configure time:
#   - a5021/stm32-async-1wire, tag v3.1.2
#   - ARM-software/CMSIS_5 (Core headers, pinned SHA)
#   - STMicroelectronics/cmsis_device_f0 (device headers, pinned SHA)
#
# The pinned CMSIS SHAs match the library's config/cmsis_pins.mk, so a fresh
# clone resolves byte-identical headers.
#
# Build:
#   cmake -S . -B build \
#       -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake \
#       -DHOST_IF=UART
#   cmake --build build -j
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(owbridge C ASM)

# LTO needs the gcc-ar wrapper (plain ar does not index LTO objects).
set(CMAKE_AR "arm-none-eabi-gcc-ar" CACHE FILEPATH "LTO-capable archiver" FORCE)
set(CMAKE_RANLIB "arm-none-eabi-gcc-ranlib" CACHE FILEPATH "LTO-capable ranlib" FORCE)

# Bare-metal link rule (no PE flags, no system libs).
set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
set(CMAKE_C_STANDARD_LIBRARIES "")
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
set(HOST_IF "UART" CACHE STRING "Host interface: UART, SPI, I2C")
set_property(CACHE HOST_IF PROPERTY STRINGS UART SPI I2C)

set(OW_MCU "STM32F030x6" CACHE STRING "ST device macro")
set(OW_SYSCLK_MHZ "48" CACHE STRING "System clock in MHz")
set(OW_MAX_DEVICES "16" CACHE STRING "DS18B20 device table size")
set(OW_PARASITE ON CACHE BOOL "Parasite-powered 1-Wire bus")
set(OW_LIBRARY_TAG "v3.1.2" CACHE STRING "stm32-async-1wire git tag")
set(OW_OPT "-O2" CACHE STRING "Optimisation flag for the firmware and the library")
option(OWBRIDGE_PROFILE "Enable the one-off I2C polling profiler (TIM3, SWD-readable)" OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
include(FetchContent)

# CMSIS Core: the pin from stm32-async-1wire config/cmsis_pins.mk.
set(CMSIS_CORE_REF "2b7495b8535bdcb306dac29b9ded4cfb679d7e5c")
# CMSIS device F0: the same pin table, CMSIS_DEVICE_REF_F0.
set(CMSIS_DEVICE_F0_REF "1ed05aa865aab1db2db2287077080aa44344fafa")

FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_5.git
    GIT_TAG        ${CMSIS_CORE_REF}
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE)

FetchContent_Declare(cmsis_device_f0
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_f0.git
    GIT_TAG        ${CMSIS_DEVICE_F0_REF}
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE)

FetchContent_Declare(stm32_async_1wire
    GIT_REPOSITORY https://github.com/a5021/stm32-async-1wire.git
    GIT_TAG        ${OW_LIBRARY_TAG}
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE)

# Populate only (do not add_subdirectory the vendors: the library's
# ow_add_library.cmake helper compiles exactly the sources we need).
foreach(_dep cmsis_core cmsis_device_f0 stm32_async_1wire)
    FetchContent_GetProperties(${_dep})
    if(NOT ${_dep}_POPULATED)
        FetchContent_Populate(${_dep})
    endif()
endforeach()

set(CMSIS_CORE_INCLUDE   "${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include")
set(CMSIS_DEVICE_INCLUDE "${cmsis_device_f0_SOURCE_DIR}/Include")
set(CMSIS_SYSTEM_SRC     "${cmsis_device_f0_SOURCE_DIR}/Source/Templates/system_stm32f0xx.c")
set(CMSIS_STARTUP_ASM    "${cmsis_device_f0_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f030x6.s")
set(OW_LINKER_SCRIPT     "${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F030X6_FLASH.ld")

# ---------------------------------------------------------------------------
# Library (sources fetched into _deps; helper script from the library)
# ---------------------------------------------------------------------------
include(${stm32_async_1wire_SOURCE_DIR}/cmake/ow_add_library.cmake)

set(_ow_lib_args
    MCU ${OW_MCU}
    SYSCLK_MHZ ${OW_SYSCLK_MHZ}
    MAX_DEVICES ${OW_MAX_DEVICES}
    CMSIS_INCLUDE_DIRS ${CMSIS_CORE_INCLUDE} ${CMSIS_DEVICE_INCLUDE}
    SOURCE_DIR ${stm32_async_1wire_SOURCE_DIR}
)
if(OW_PARASITE)
    list(APPEND _ow_lib_args PARASITE)
endif()
ow_add_library(owlib ${_ow_lib_args})

target_compile_options(owlib PRIVATE ${OW_OPT} -flto)
if(OWBRIDGE_PROFILE)
    # Let the profiled build time the driver's real work events (OW_PROFILE
    # hooks); the application provides the two functions in src/profile.c.
    target_compile_definitions(owlib PRIVATE OW_PROFILE=1)
endif()
target_link_options(owlib INTERFACE -flto)

# ---------------------------------------------------------------------------
# Host interface selection (compile-time)
# ---------------------------------------------------------------------------
if(HOST_IF STREQUAL "UART")
    set(HOST_SRC src/host_uart.c)
    add_compile_definitions(HOST_IF_UART=1)
elseif(HOST_IF STREQUAL "SPI")
    set(HOST_SRC src/host_spi.c)
    add_compile_definitions(HOST_IF_SPI=1)
elseif(HOST_IF STREQUAL "I2C")
    set(HOST_SRC src/host_i2c_soft.c)
    add_compile_definitions(HOST_IF_I2C=1)
else()
    message(FATAL_ERROR "HOST_IF must be one of: UART, SPI, I2C")
endif()

# ---------------------------------------------------------------------------
# Firmware
# ---------------------------------------------------------------------------
if(OWBRIDGE_PROFILE)
    add_compile_definitions(OWBRIDGE_PROFILE=1)
endif()

add_executable(owbridge
    src/main.c
    src/board.c
    src/bridge.c
    src/protocol.c
    src/cache.c
    src/config.c
    src/stats.c
    src/profile.c
    ${HOST_SRC}
    ${CMSIS_SYSTEM_SRC}
    ${CMSIS_STARTUP_ASM}
)
target_include_directories(owbridge PRIVATE
    include
    src
    ${CMSIS_CORE_INCLUDE}
    ${CMSIS_DEVICE_INCLUDE}
)
# Flash is plentiful (31 KB code region), so the firmware - not the fetched
# library - gets the hot-path options: aligned functions/loops and no
# loop-to-memcpy conversion. The library keeps OW_OPT; its timing is
# hardware-driven. A firmware-only -O3 build fits but was slower on the
# Cortex-M0, so it is deliberately not used.
target_compile_options(owbridge PRIVATE ${OW_OPT} -flto
    -ffunction-sections -fdata-sections
    -falign-functions=4 -falign-loops=4
    # Keep the explicit word/4-byte loops in fast_copy(): GCC otherwise
    # recognises them as memcpy and calls the slow byte-wise libc routine.
    -fno-tree-loop-distribute-patterns)
target_link_libraries(owbridge PRIVATE owlib)
target_link_options(owbridge PRIVATE
    -T${OW_LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--no-warn-rwx-segments
    -Wl,-Map=${CMAKE_BINARY_DIR}/owbridge.map
    --specs=nano.specs
    -lc -lm -lnosys
)

# Flashable images next to the ELF (the README documents these outputs).
add_custom_command(TARGET owbridge POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:owbridge>
            ${CMAKE_BINARY_DIR}/owbridge.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:owbridge>
            ${CMAKE_BINARY_DIR}/owbridge.hex
    COMMENT "Generating owbridge.bin and owbridge.hex"
)
