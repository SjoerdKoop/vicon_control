cmake_minimum_required(VERSION 2.8.3)
project(vision_control)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp roslaunch vision_control_tools)


## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ../vision_control_tools/include)


## Create package
catkin_package(CATKIN_DEPENDS vision_control_tools)


## Add executables

# Main program
add_executable(vision_control src/${PROJECT_NAME}/vision_control.cpp src/${PROJECT_NAME}/example_controller.cpp)
target_link_libraries(vision_control ${catkin_LIBRARIES} vision_control_tools)