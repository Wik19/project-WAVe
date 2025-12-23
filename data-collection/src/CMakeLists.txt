# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# idf_component_register(SRCS ${app_sources})

idf_component_register(
    SRCS 
        "src/main.c"
        "src/wifi_conn.c"
        "src/imu_driver.c"
        "src/tcp_server.c"
    
    INCLUDE_DIRS 
        "include"
)
