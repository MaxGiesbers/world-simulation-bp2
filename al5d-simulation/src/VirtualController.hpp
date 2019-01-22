
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include "VirtualServo.hpp"

class VirtualController
{
private:
  char command_character;
  std::vector<char> delimeters = { '#', 'P', 'S', 'T', '\r' };
  ros::NodeHandle n;
  ros::Subscriber msg_subscriber;
  VirtualServo servo;

  void parseMessage(const std_msgs::String& msg);
  void setFirstAndSecondCharPosition(const std::string& message, short& first_position, short& second_position);
  short getFirstIndexPosition(const std::string& message);

public:
  VirtualController();
  ~VirtualController();
};