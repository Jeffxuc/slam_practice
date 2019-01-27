cmake_minimum_required(VERSION 3.13)
project(3_geometric_train)

set(CMAKE_CXX_STANDARD 11)

include_directories(/usr/local/include/eigen3)

add_executable(3_geometric_train main.cpp)