# CMakeLists.txt for ESP32-C3 MP3 Streamer
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_mp3_streamer)

set(helix_sources
    "helix/mp3dec.c"
    "helix/mp3tabs.c"
    "helix/bitstream.c"
    "helix/buffers.c"
    "helix/dct32.c"
    "helix/dequant.c"
    "helix/dqchan.c"
    "helix/huffman.c"
    "helix/hufftabs.c"
    "helix/imdct.c"
    "helix/scalfact.c"
    "helix/stproc.c"
    "helix/subband.c"
    "helix/trigtabs.c"
)

idf_component_register(
    SRCS "main.c" ${helix_sources}
    INCLUDE_DIRS "." "helix"
    REQUIRES esp_http_client nvs_flash esp_wifi driver
)