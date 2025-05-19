cmake_minimum_required(VERSION 3.16)

# Add the components directory to the search path
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

# Bluetooth options
set(COMPONENT_REQUIRES bt esp_wifi)

# Include external headers
set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)
project(glove-system)