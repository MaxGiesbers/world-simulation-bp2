
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "al5d_simulation/servo_command.h"
#include <vector>
#include "VirtualServo.hpp"
#include <algorithm>
#include "LynxMotionSimulator.hpp"

class VirtualController
{
private:
  char command_character;
  std::vector<char> delimeters = { '#', 'P', 'S', 'T', '\r' };
  ros::NodeHandle n;
  ros::Subscriber msg_subscriber;
  std::vector<VirtualServo> servo_list;
  al5d_simulation::servo_command servo_degrees_msg;
  
  void parseMessage(const std_msgs::String& msg);
  void setFirstAndSecondCharPosition(const std::string& message, short& first_position, short& second_position);
  void initServoList();
  short getFirstIndexPosition(const std::string& message);
  void publishMessage(const VirtualServo& servo);

public:
  VirtualController(const Position& robot_arm_position);

  ~VirtualController();
  ros::Publisher servo_degrees_publisher;
};