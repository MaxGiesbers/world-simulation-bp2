cmake_minimum_required(VERSION 2.8.3)
project(al5d_simulation)

## Compile as C++14, supported in ROS Kinetic and newer
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
  lynx_motion_simulator
  src/virtual_controller.cpp 
  src/virtual_servo.cpp
  src/lynx_motion_simulator.cpp)

add_dependencies(lynx_motion_simulator al5d_simulation_generate_messages_cpp)
target_link_libraries(lynx_motion_simulator ${catkin_LIBRARIES})


add_executable(publisher src/publisher.cpp)
add_dependencies(publisher al5d_simulation_generate_messages_cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})


add_executable(
cup_simulator
src/cup_simulator.cpp
src/cup.cpp)
add_dependencies(cup_simulator al5d_simulation_generate_messages_cpp)
target_link_libraries(cup_simulator ${catkin_LIBRARIES})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)








