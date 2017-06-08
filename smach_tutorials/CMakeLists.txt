cmake_minimum_required(VERSION 2.8.3)
project(smach_tutorials)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation)

add_action_files(
  DIRECTORY action
  FILES Test.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs message_runtime)

install(
  DIRECTORY examples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)