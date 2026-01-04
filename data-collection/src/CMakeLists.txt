# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# idf_component_register(SRCS ${app_sources})

idf_component_register(
    SRCS 
        "main.c"
        "imu_driver.c"    # Added this
        "wifi_connect.c"  # Added this

    INCLUDE_DIRS 
        "../include"      # Keeps looking in your global include folder
        "."               # Also looks in the current folder for .h files
)