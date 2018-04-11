## Set project properties
cmake_minimum_required(VERSION 2.8.3)
project(components)
add_compile_options(-std=c++11)

## Set include paths
include_directories(include)

## Create components library
add_library(${PROJECT_NAME} src/peer.cpp src/pru.cpp)