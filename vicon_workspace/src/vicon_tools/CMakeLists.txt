cmake_minimum_required(VERSION 2.8.3)
project(vicon_tools)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)


## Add and generate messages
add_message_files(FILES
	remove_objects.msg
	ros_object.msg
	ros_object_array.msg
)

generate_messages()


## Create package
catkin_package(CATKIN_DEPENDS message_runtime)


## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS} include ${CMAKE_SOURCE_DIR}/vicon_datastream_sdk)


## Specify additional locations of libraries
link_directories(${CMAKE_SOURCE_DIR}/vicon_datastream_sdk)


## Declare C++ libraries

# TrackedObject library
add_library(tracked_object src/${PROJECT_NAME}/tracked_object.cpp)

# Vector3D library
add_library(vector3D src/${PROJECT_NAME}/vector3D.cpp)

# General library
add_library(general_tools src/${PROJECT_NAME}/general_tools.cpp)
target_link_libraries(tracked_object vector3D)

# ViconClient (and derivatives) library
add_library(clients src/${PROJECT_NAME}/clients.cpp)
target_link_libraries(clients tracked_object vector3D ViconDataStreamSDK_CPP)


## Add executables

# Marker client program
add_executable(markers src/${PROJECT_NAME}/markers.cpp)
add_dependencies(markers ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(markers ${catkin_LIBRARIES} clients general_tools)

# Object client program
add_executable(objects src/${PROJECT_NAME}/objects.cpp)
add_dependencies(objects ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(objects ${catkin_LIBRARIES} clients general_tools)