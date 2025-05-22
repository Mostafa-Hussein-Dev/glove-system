cmake_minimum_required(VERSION 3.16)

# Include the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Register components in the `components/` directory
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/components")

# Define the project
project(glove-system)

# Optional: Suppress specific warning (used for printf-style format mismatch)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format" APPEND)
