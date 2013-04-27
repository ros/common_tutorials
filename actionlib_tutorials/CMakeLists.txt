cmake_minimum_required(VERSION 2.8.3)
project(learning_actionlib)

find_package(catkin REQUIRED COMPONENTS actionlib message_generation) 

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(DIRECTORY action FILES Fibonacci.action # Averaging.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_actionlib
  CATKIN_DEPENDS roscpp actionlib message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(fibonacci_server src/fibonacci_server.cpp)
add_dependencies(fibonacci_server learning_actionlib_gencpp)
target_link_libraries(fibonacci_server ${catkin_LIBRARIES})

add_executable(fibonacci_client src/fibonacci_client.cpp)
add_dependencies(fibonacci_client learning_actionlib_gencpp)
target_link_libraries(fibonacci_client ${catkin_LIBRARIES})

#add_executable(averaging_server src/averaging_server.cpp)
#target_link_libraries(averaging_server ${catkin_LIBRARIES})

#add_executable(averaging_client src/averaging_client.cpp)
#target_link_libraries(averaging_client ${catkin_LIBRARIES})
