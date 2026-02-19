# SPDX-License-Identifier: Apache-2.0

if(CONFIG_FPU)
    set(DWTLIBNAME libdwt_uwb_driver-m4-hfp-6.0.14.a)
else()
    set(DWTLIBNAME libdwt_uwb_driver-m4-sfp-6.0.14.a)
endif()

set(DWTLIB_FULLPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/${DWTLIBNAME})

target_link_options(zephyr_interface INTERFACE
    "-Wl,--whole-archive,${DWTLIB_FULLPATH},--no-whole-archive"
)

zephyr_include_directories(inc)

zephyr_linker_sources(SECTIONS custom-sections.ld)
