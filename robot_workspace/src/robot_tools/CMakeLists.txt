cmake_minimum_required(VERSION 2.8.3)
project(robot_tools)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)


## Add and generate messages
add_message_files(FILES data_update.msg data_update_array.msg)
generate_messages()


## Create package
catkin_package(CATKIN_DEPENDS message_runtime)


## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})


## Declare C++ libraries

# Data client library
add_library(data_client src/${PROJECT_NAME}/data_client.cpp)
add_dependencies(data_client ${PROJECT_NAME}_generate_messages_cpp)

# Peer library
add_library(peer src/${PROJECT_NAME}/peer.cpp)

# Reference server library
add_library(reference_server src/${PROJECT_NAME}/reference_server.cpp)
add_dependencies(reference_server ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(reference_server PRIVATE ${catkin_LIBRARIES} peer)

# Tools library
add_library(tools src/${PROJECT_NAME}/tools.cpp)


## Add executables

# Communicate program
add_executable(communicate src/${PROJECT_NAME}/communicate.cpp)
target_link_libraries(communicate ${catkin_LIBRARIES} data_client peer reference_server tools)

# Data subscriber program
add_executable(data_subscriber src/${PROJECT_NAME}/data_subscriber.cpp)
add_dependencies(data_subscriber ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(data_subscriber ${catkin_LIBRARIES})

# Send reference program
add_executable(send_reference src/${PROJECT_NAME}/send_reference.cpp)
target_link_libraries(send_reference ${catkin_LIBRARIES} peer tools)