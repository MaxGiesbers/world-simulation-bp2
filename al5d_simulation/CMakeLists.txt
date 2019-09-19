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
  servo_position.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
  lynx_motion_simulator
  src/al5d_simulator/virtual_controller.cpp 
  src/al5d_simulator/lynx_motion_simulator.cpp)

add_dependencies(lynx_motion_simulator al5d_simulation_generate_messages_cpp)
target_link_libraries(lynx_motion_simulator ${catkin_LIBRARIES})

add_executable(
  virtual_servo
  src/al5d_simulator/virtual_servo.cpp
)

add_dependencies(virtual_servo al5d_simulation_generate_messages_cpp)
target_link_libraries(virtual_servo ${catkin_LIBRARIES})

add_executable(demo_publish_node src/demo_publisher/demo_publish_node.cpp)
add_dependencies(demo_publish_node al5d_simulation_generate_messages_cpp)
target_link_libraries(demo_publish_node ${catkin_LIBRARIES})

add_executable(
cup_simulator
src/cup_simulator/cup_simulator.cpp
src/cup_simulator/cup.cpp)
add_dependencies(cup_simulator al5d_simulation_generate_messages_cpp)
target_link_libraries(cup_simulator ${catkin_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS})