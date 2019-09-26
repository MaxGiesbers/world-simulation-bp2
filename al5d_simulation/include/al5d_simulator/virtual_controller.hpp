#include "ros/ros.h"
#include "std_msgs/String.h"
#include "al5d_simulation/servo_command.h"
#include "lynx_motion_simulator.hpp"

class VirtualController
{
private:
  uint8_t m_command_character;
  std::vector<char> delimeters = { '#', 'P', 'S', 'T', '\r' };
  ros::NodeHandle m_node_handle;
  ros::Subscriber m_subscriber;
  al5d_simulation::servo_command m_servo_message;
  ros::Publisher m_publisher;

  void parseMessage(const std_msgs::String& incoming_message);
  void setFirstAndSecondCharPosition(const std::string& message, uint8_t& first_position, uint8_t& second_position);
  short getFirstIndexPosition(const std::string& message);

public:
  VirtualController(const Position& robot_arm_position);
  ~VirtualController();
};