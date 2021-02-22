cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Harrisdetect)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (Harrisdetect Harrisdetect.cpp)
target_link_libraries (Harrisdetect ${PCL_LIBRARIES})
 

