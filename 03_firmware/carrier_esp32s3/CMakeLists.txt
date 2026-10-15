cmake_minimum_required(VERSION 3.16)
list(APPEND EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../common ${CMAKE_CURRENT_LIST_DIR}/../carrier_core)   # APPEND: PlatformIO adds src/ here
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(atlas_carrier)
