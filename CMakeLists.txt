cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Register components in the `components/` directory
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/components")

# Define the project
project(glove-system)

# Optional: Suppress specific warning (used for printf-style format mismatch)
idf_build_set_property(COMPILE_OPTIONS "-Wno-format" APPEND)


# Add C++ standard library paths
list(APPEND CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES
    "${CMAKE_SYSROOT}/usr/include/c++/v1"
    "${CMAKE_SYSROOT}/usr/include"
)

# Force include C++ headers
idf_build_set_property(COMPILE_OPTIONS "-I${IDF_PATH}/tools/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/8.4.0" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-I${IDF_PATH}/tools/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/8.4.0/xtensa-esp32s3-elf" APPEND)

# spiffs_create_partition_image(spiffs data/ FLASH_IN_PROJECT)