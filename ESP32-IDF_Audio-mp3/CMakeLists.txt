# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Add ESP-ADF components path
set(ADF_PATH "/home/alex/.espressif/esp-adf")  # Change this to your ESP-ADF installation path
if(NOT ADF_PATH)
    message(FATAL_ERROR "ADF_PATH is not set. Please set it to your ESP-ADF installation directory.")
endif()

set(EXTRA_COMPONENT_DIRS ${ADF_PATH}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-IDF_Audio-mp3)
