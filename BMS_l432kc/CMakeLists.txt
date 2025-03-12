# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(BMS_l432kc)

target_sources(app PRIVATE source/main.c)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include)

