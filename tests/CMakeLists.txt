cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_CAN_Bus)

target_sources(app PRIVATE test_CAN_Bus.c)
