cmake_minimum_required(VERSION 2.8.3)
project(vicon_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

## Add and generate messages
add_message_files(FILES remove_objects.msg ros_object.msg ros_object_array.msg)
generate_messages()

## Create package
catkin_package(CATKIN_DEPENDS message_runtime)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare C++ libraries
add_library(tracked_object src/${PROJECT_NAME}/tracked_object.cpp)
add_library(vector3D src/${PROJECT_NAME}/vector3D.cpp)
add_library(vicon_tools src/${PROJECT_NAME}/vicon_tools.cpp)
target_link_libraries(tracked_object vector3D)

add_library(clients src/${PROJECT_NAME}/clients.cpp)
target_link_libraries(clients tracked_object vector3D ViconDataStreamSDK_CPP vicon_tools)

## Add executables
add_executable(markers src/${PROJECT_NAME}/markers.cpp)
add_dependencies(markers ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(markers ${catkin_LIBRARIES} clients vicon_tools)


add_executable(objects src/${PROJECT_NAME}/objects.cpp)
add_dependencies(objects ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(objects ${catkin_LIBRARIES} clients vicon_tools)