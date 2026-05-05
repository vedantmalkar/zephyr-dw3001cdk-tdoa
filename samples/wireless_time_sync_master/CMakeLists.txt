cmake_minimum_required(VERSION 3.20)

set(BOARD decawave_dwm3001cdk)

set(DTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
list(APPEND DTS_ROOT "${REPO_ROOT}")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wireless_time_sync_master)

add_subdirectory(../../drivers/dw3000 dw3000)

target_include_directories(app PRIVATE
    ../../drivers/dw3000/inc
    ../../drivers/platform
)

target_sources(app PRIVATE
    src/main.c
    ../../drivers/platform/dw3000_spi.c
    ../../drivers/platform/dw3000_hw.c
    ../../drivers/platform/port.c
    ../../drivers/platform/deca_port.c
)
