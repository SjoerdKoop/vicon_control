## Set project properties
cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

## Set general include directory
include_directories(include)

## Set COMPONENTS_INCLUDE_DIR to point to components headers
set(COMPONENTS_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src/components/include)

## Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

## Add subdirectories (which include CMakeLists.txt)
#add_subdirectory(robot)
#add_subdirectory(robot_test)
add_subdirectory(src/components)
add_subdirectory(src/tools)