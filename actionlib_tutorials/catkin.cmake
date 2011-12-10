project(actionlib_tutorials)

include_directories(${actionlib_INCLUDE_DIRS} ${ROS_INCLUDE_DIRS})

add_action_files(
  DIRECTORY action
  FILES Averaging.action Fibonacci.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

add_executable(fibonacci_client simple_action_clients/fibonacci_client.cpp)
add_executable(averaging_client simple_action_clients/averaging_client.cpp)

target_link_libraries(fibonacci_client ${ROS_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(averaging_client ${ROS_LIBRARIES} ${Boost_LIBRARIES})
