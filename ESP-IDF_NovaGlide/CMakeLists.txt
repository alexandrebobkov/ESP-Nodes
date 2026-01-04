cmake_minimum_required(VERSION 3.16)

# Add subsystems to component search path
set(EXTRA_COMPONENT_DIRS
    "subsystems/connectivity/wifi_sys"
    "subsystems/connectivity/mqtt_sys"
    "subsystems/connectivity/espnow_sys"
    "subsystems/motors"
    "subsystems/adc"
    "subsystems/sensors"
    "subsystems/controls"
    "subsystems/ui")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP-IDF_NovaGlide)
