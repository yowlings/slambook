cmake_minimum_required(VERSION 2.8)

project(ch2)
add_executable(${PROJECT_NAME} "main.cpp")

add_library(hello_shared SHARED uselib.cpp)
add_executable(libHello libhello.cpp)
target_link_libraries(libHello hello_shared)
