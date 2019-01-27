cmake_minimum_required(VERSION 3.13)
project(2_gxslam)

set(CMAKE_CXX_STANDARD 11)
include_directories(/usr/include/eigen3)
add_executable(2_gxslam main.cpp)