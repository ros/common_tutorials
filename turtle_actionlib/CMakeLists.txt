cmake_minimum_required(VERSION 2.8.3)
project(turtle_actionlib)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs angles message_generation rosconsole roscpp std_msgs turtlesim)

add_message_files(DIRECTORY msg FILES Velocity.msg)
add_action_files(DIRECTORY action FILES Shape.action)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES turtle_actionlib_tutorial
  CATKIN_DEPENDS actionlib message_generation roscpp
#  DEPENDS system_lib
)

#include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(shape_server src/shape_server.cpp )
add_dependencies(shape_server thread)
target_link_libraries(shape_server ${catkin_LIBRARIES})

add_executable(shape_client src/shape_client.cpp)
add_dependencies(shape_client thread)
target_link_libraries(shape_client ${catkin_LIBRARIES})
