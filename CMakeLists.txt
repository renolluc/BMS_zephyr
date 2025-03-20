# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app)

target_sources(app
        PRIVATE
        source/main.c
        source/modules/CAN_Bus.c
        
)

zephyr_include_directories(include)
