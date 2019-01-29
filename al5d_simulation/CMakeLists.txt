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

# add_executable(LynxMotionSimulator src/LynxMotionSimulator.cpp)
# add_dependencies(LynxMotionSimulator al5d_simulation_generate_messages_cpp)
# target_link_libraries(LynxMotionSimulator ${catkin_LIBRARIES})


add_executable(
  LynxMotionSimulator 
  src/VirtualController.cpp 
  src/VirtualServo.cpp
  src/LynxMotionSimulator.cpp)

add_dependencies(LynxMotionSimulator al5d_simulation_generate_messages_cpp)
target_link_libraries(LynxMotionSimulator ${catkin_LIBRARIES})


add_executable(Publisher src/Publisher.cpp)
add_dependencies(Publisher al5d_simulation_generate_messages_cpp)
target_link_libraries(Publisher ${catkin_LIBRARIES})


add_executable(
CupSimulator
src/CupSimulator.cpp
src/Cup.cpp)
add_dependencies(CupSimulator al5d_simulation_generate_messages_cpp)
target_link_libraries(CupSimulator ${catkin_LIBRARIES})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)








