#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

set(BOARD nicoco)
list(APPEND EXTRA_CONF_FILE "overlay_usb_mass_storage.conf overlay_smp_bt.conf")

include(secrets.cmake OPTIONAL)
if("${EI_API_KEY_HEADER}" MATCHES "^x-api-key:ei_[0-9a-fA-F]+$") # Cannot specify [0-9a-fA-F]{64} because of CMake regex limitations
    message(VERBOSE "Edge Impulse API key is set")
    list(APPEND EXTRA_CONF_FILE "overlay_edge_impulse.conf")
else()
    message(WARNING "Edge Impulse API key is not set")
endif()

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_include_directories(app PRIVATE src)

add_subdirectory_ifdef(CONFIG_USB_MASS_STORAGE src/usb_mass_storage)
add_subdirectory(src/state_machine)
add_subdirectory(src/battery)
add_subdirectory_ifdef(CONFIG_EDGE_IMPULSE src/edge-impulse)

target_sources(app PRIVATE src/main.c)
