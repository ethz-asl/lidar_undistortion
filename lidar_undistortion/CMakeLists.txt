cmake_minimum_required(VERSION 2.8.3)
project(lidar_undistortion)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(lidar_undistortion_node
    src/lidar_undistortion_node.cpp
    src/lidar_undistorter.cpp)

cs_install()
cs_export()
