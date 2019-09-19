#include "ros/ros.h"
#include "al5d_simulation/servo_position.h"
#include "al5d_simulation/servo_command.h"

class VirtualServo
{
private:
  uint8_t m_channel;
  uint16_t m_movement_speed;
  uint16_t m_time;
  double m_current_degrees;
  ros::Publisher m_publisher;
  ros::NodeHandle m_node_handle;
  ros::Subscriber m_subscriber;
  al5d_simulation::servo_position m_servo_message;

  void callBack(const al5d_simulation::servo_command& servo);
  void updateServoPosition(double incoming_degrees, double movement_speed);
  void publishMessage(const al5d_simulation::servo_command& servo);
  double mapGripper(uint16_t incoming_pwm);
  double mapServo(uint16_t incoming_pwm);
  bool almostEquals(double a, double b);

public:
  VirtualServo(uint8_t channel);
  ~VirtualServo();
};