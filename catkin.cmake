cmake_minimum_required(VERSION 2.8)
project(common_tutorials)
find_package(catkin)
find_package(actionlib)

find_package(ROS REQUIRED COMPONENTS catkin roscpp_serialization std_msgs sensor_msgs roscpp actionlib rosconsole rostime cpp_common roscpp_traits)
find_package(Boost REQUIRED COMPONENTS python)

foreach(subdir
    actionlib_tutorials
    )
  add_subdirectory(${subdir})
endforeach()

catkin_package(common_tutorials)
