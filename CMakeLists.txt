# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Avoid pulling in the legacy in-tree "freemodbus" component, which conflicts
# with the espressif/esp-modbus managed component used by main.
set(EXCLUDE_COMPONENTS freemodbus)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(t-vending)
