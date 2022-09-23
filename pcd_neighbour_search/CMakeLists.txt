cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_downsample)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_downsample pcd_downsample.cpp)
target_link_libraries (pcd_downsample ${PCL_LIBRARIES})