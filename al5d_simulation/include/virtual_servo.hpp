#include "ros/ros.h"
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

class VirtualServo
{
public:
  VirtualServo();
  ~VirtualServo();
  VirtualServo(const VirtualServo& a_servo);

  void setChannel(short a_channel);
  void setIncomingPwm(const short a_incoming_pwm);
  void setCurrentDegrees(const double a_current_degrees);
  void setMovementSpeed(const short a_movement_speed);
  void setTime(const short a_time);

  short getChannel() const;
  double getCurrentDegrees() const;
  short getIncomingPWM() const;
  double pwmToDegrees();
  double mapGripper();

private:
  short channel;
  short incoming_pwm;
  short movement_speed;
  short time;
  double current_degrees;
};