cmake_minimum_required(VERSION 3.13)
project(disparity)

set(CMAKE_CXX_STANDARD 11)

#OpenCV为常用的库文件，由于有OpenCV 2 和OpenCV 3这两个版本存在，当电脑里面安装了两个版本时，
#在find_package时要指定查找的OpenCV是哪个版本的。（版本号与OpenCV之间有空格）
#同时最重要的是要注意OpenCV的大小写（非常重要，常常弄错）。
#头文件路径为：${OpenCV_INCLUDE_DIRS}，库文件路径为：${OpenCV_LIBS}。
find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#Pangolin是一个轻量级的OpenGL输入/输出和视频显示的库，可以显示２Ｄ/3D的图。
#其头文件路径为：${Pangolin_INCLUDE_DIRS}，
#链接库文件的路径为：${Pangolin_LIBRARIES}，注意其后部分与OpenCV的库文件变量名不同。
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS} )

#Eigen为开源C++线性代数库，提供矩阵和线性代数的运算。
#由于eigen库只有头文件，没有 .so或 .a那样的二进制文件，所以不需要 target_link_libraries。
#且通过apt安装后，其默认路径为: /usr/include/eigen3
include_directories(/usr/local/include/eigen3)

add_executable(disparity disparity.cpp)

target_link_libraries(disparity
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        )
