cmake_minimum_required(VERSION 2.8.3)
project(al5d-simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)








