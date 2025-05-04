cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Define the test project
project(test_app)

# Add the test source file
target_sources(app PRIVATE 
    #${CMAKE_CURRENT_SOURCE_DIR}/test_spi_mb.c
    #${CMAKE_CURRENT_SOURCE_DIR}/test_CAN_Bus.c
    #${CMAKE_CURRENT_SOURCE_DIR}/test_battery.c
    #${CMAKE_CURRENT_SOURCE_DIR}/test_shutdown_circuit.c
)

zephyr_include_directories(./include)