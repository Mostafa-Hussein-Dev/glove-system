cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)
project(glove-system)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format" APPEND)