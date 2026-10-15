cmake_minimum_required(VERSION 3.16)
# shared, host-tested code: protocol + PI + docking FSM (common), agent decisions (agent_core)
list(APPEND EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../common ${CMAKE_CURRENT_LIST_DIR}/../agent_core)   # APPEND: PlatformIO adds src/ here
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(atlas_agent)
