cmake_minimum_required(VERSION 3.16)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(epaper_lvgl_demo)

if(LVGL_VERSION_MAJOR GREATER 8)
    idf_build_set_property(COMPILE_OPTIONS "-DLV_CONF_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h\"" APPEND)
else()
    idf_build_set_property(COMPILE_OPTIONS "-DLV_CONF_PATH=${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h" APPEND)
endif()
