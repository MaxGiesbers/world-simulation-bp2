#include "ros/ros.h"
#include <chrono>
#include <thread>
#include "al5d_simulation/servo_command.h"
using namespace std::chrono_literals;

class VirtualServo
{
public:
  VirtualServo(short a_channel = 0);
  ~VirtualServo();

  void setChannel(short a_channel);
  void setIncomingPwm(short a_incoming_pwm);
  void setMovementSpeed(short a_movement_speed);
  void setTime(short a_time);
  
  void publishMessage();

  short getChannel() const;
  short current_degrees;
  short channel;
  short incoming_pwm;
  short movement_speed;
  short time;

  private:
  // short channel;
  // short incoming_pwm;
  // short movement_speed;
  // short time;
  
  short pwmToDegrees ();
  ros::Publisher servo_degrees_publisher;
  ros::NodeHandle n;
  al5d_simulation::servo_command servo_degrees_msg;
};