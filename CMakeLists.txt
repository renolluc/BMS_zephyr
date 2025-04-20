# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(BOARD "nucleo_l432kc")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


project(app)

target_sources(app
        PRIVATE
        source/modules/CAN_Bus.c        
        source/modules/SPI_MB.c
        source/modules/serial_monitor.c
)


zephyr_include_directories(./include)


if (NOT CONFIG_ZTEST)
    target_sources(app PRIVATE source/main.c)
endif()

if (CONFIG_ZTEST)
    add_subdirectory(tests)
endif()