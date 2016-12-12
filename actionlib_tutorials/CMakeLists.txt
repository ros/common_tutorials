cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS actionlib message_generation)

## Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)

## Add actions
add_action_files(DIRECTORY action FILES Fibonacci.action Averaging.action)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp actionlib
  DEPENDS Boost
)

## Setup include dirs
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Averaging server
add_executable(averaging_server src/averaging_server.cpp)
target_link_libraries(averaging_server ${catkin_LIBRARIES})
add_dependencies(averaging_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Averaging client
add_executable(averaging_client src/averaging_client.cpp)
target_link_libraries(averaging_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(averaging_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Fibonacci server
add_executable(fibonacci_server src/fibonacci_server.cpp)
target_link_libraries(fibonacci_server ${catkin_LIBRARIES})
add_dependencies(fibonacci_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Fibonacci clients
add_executable(fibonacci_client src/fibonacci_client.cpp)
add_executable(fibonacci_callback_client src/fibonacci_callback_client.cpp)
add_executable(fibonacci_class_client src/fibonacci_class_client.cpp)
target_link_libraries(fibonacci_client ${catkin_LIBRARIES})
target_link_libraries(fibonacci_callback_client ${catkin_LIBRARIES})
target_link_libraries(fibonacci_class_client ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(fibonacci_client ${catkin_EXPORTED_TARGETS})
  add_dependencies(fibonacci_callback_client ${catkin_EXPORTED_TARGETS})
  add_dependencies(fibonacci_class_client ${catkin_EXPORTED_TARGETS})
endif()
add_dependencies(fibonacci_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fibonacci_callback_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fibonacci_class_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install scripts and executables
install(PROGRAMS
  scripts/fibonacci_client.py
  scripts/fibonacci_server.py
  scripts/gen_numbers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS averaging_server averaging_client fibonacci_server fibonacci_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

