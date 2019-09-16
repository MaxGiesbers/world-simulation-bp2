
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "al5d_simulation/servo_command.h"
#include <vector>
//#include "virtual_servo.hpp"
#include <algorithm>
#include "lynx_motion_simulator.hpp"

class VirtualController
{
private:
  char command_character;
  std::vector<char> delimeters = { '#', 'P', 'S', 'T', '\r' };
  ros::NodeHandle n;
  ros::Subscriber msg_subscriber;
  al5d_simulation::servo_command servo_degrees_msg;


  void parseMessage(const std_msgs::String& msg);
  void setFirstAndSecondCharPosition(const std::string& message, short& first_position, short& second_position);
  short getFirstIndexPosition(const std::string& message);

public:
  VirtualController(const Position& robot_arm_position);

  ~VirtualController();
  ros::Publisher servo_publisher;
};