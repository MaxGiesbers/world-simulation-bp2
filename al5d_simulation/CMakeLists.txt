cmake_minimum_required(VERSION 2.8.3)
project(al5d_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)


add_message_files(
  FILES
  servo_command.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()





include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(LynxMotionSimulator src/LynxMotionSimulator.cpp)
add_dependencies(LynxMotionSimulator al5d_simulation_generate_messages_cpp)
target_link_libraries(LynxMotionSimulator ${catkin_LIBRARIES})


add_executable(
  VirtualController 
  src/VirtualController.cpp 
  src/VirtualServo.cpp)

add_dependencies(VirtualController al5d_simulation_generate_messages_cpp)
target_link_libraries(VirtualController ${catkin_LIBRARIES})


add_executable(Publisher src/Publisher.cpp)
add_dependencies(Publisher al5d_simulation_generate_messages_cpp)
target_link_libraries(Publisher ${catkin_LIBRARIES})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)








