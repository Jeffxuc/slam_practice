cmake_minimum_required(VERSION 3.13)
project(undistort_image)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)

include_directories(OpenCV ${Opncv_INCLUDE_DIRS})
add_executable(undistort_image  undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS})