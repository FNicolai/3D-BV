cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCL_PROJECT)

find_package(PCL REQUIRED COMPONENTS io common visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pclproject main.cpp import_and_clean.cpp)
target_link_libraries(pclproject ${PCL_LIBRARIES})
