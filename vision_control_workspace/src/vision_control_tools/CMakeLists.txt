cmake_minimum_required(VERSION 2.8.3)
project(vision_control_tools)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)


## Add and generate messages
add_message_files(FILES
	reference.msg
	ros_object.msg
	ros_object_array.msg
)

generate_messages()


## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})


## Create package
catkin_package(CATKIN_DEPENDS message_runtime)


## Declare C++ libraries

# Package library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/vision.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)


## Add executables

# Object client program
add_executable(object_client src/${PROJECT_NAME}/object_client.cpp)
add_dependencies(object_client ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(object_client ${catkin_LIBRARIES})

# Reference server program
add_executable(reference_server src/${PROJECT_NAME}/reference_server.cpp)
add_dependencies(reference_server ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(reference_server ${catkin_LIBRARIES})