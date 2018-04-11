## Set project properties
cmake_minimum_required(VERSION 2.8.3)
project(tools)
add_compile_options(-std=c++11)

## Set include paths
include_directories(include ${COMPONENTS_INCLUDE_DIR})

## Create tools library
add_library(tools src/tools.cpp)

## Create data_server executable
add_executable(data_server src/data_server.cpp)
target_link_libraries(data_server components tools)

## Create reference_client executable
add_executable(reference_client src/reference_client.cpp)
target_link_libraries(reference_client components tools)

## Create read_shared_memory executable
add_executable(read_shared_memory src/read_shared_memory.cpp)
target_link_libraries(read_shared_memory components tools)

## Create write_shared_memory executable
add_executable(write_shared_memory src/write_shared_memory.cpp)
target_link_libraries(write_shared_memory components tools)