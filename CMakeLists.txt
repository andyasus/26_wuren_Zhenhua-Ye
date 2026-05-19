cmake_minimum_required(VERSION 3.10)
project(CMakeTest)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(common)
add_subdirectory(modules)

add_executable(test main.cpp)

target_link_libraries(test
    kalman
    math
    A1
    A2
    M1
    M2
)