cmake_minimum_required(VERSION 3.20)

set(BOARD decawave_dwm3001cdk)
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uwb_bringup)

add_subdirectory(drivers/dw3000)

target_include_directories(app PRIVATE
    drivers/dw3000/inc
    drivers/platform    
    # lib/sit
)
  
# target_link_libraries(app PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/drivers/dw3000/lib/libdwt_uwb_driver-m4-hfp-6.0.14.a
# )

target_sources(app PRIVATE
    src/main.c
    drivers/platform/dw3000_spi.c
    drivers/platform/dw3000_hw.c
    drivers/platform/port.c
    drivers/platform/deca_port.c
    # lib/sit/sit.c
    # lib/sit/sit_device.c
    # lib/sit/sit_config.c
)