cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_sd_roughness)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_sd_roughness pcd_sd_roughness.cpp)
target_link_libraries (pcd_sd_roughness ${PCL_LIBRARIES})