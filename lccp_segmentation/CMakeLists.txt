cmake_minimum_required(VERSION 2.9 FATAL_ERROR)

project(example_lccp_segmentation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(example_lccp_segmentation example_lccp_segmentation.cpp)
target_link_libraries(example_lccp_segmentation ${PCL_LIBRARIES})