#include "ros/ros.h"


class VirtualServo
{


public:
  VirtualServo(short a_channel);
  ~VirtualServo();

  void setChannel(short a_channel);
  void setIncomingPwm(short a_incoming_pwm);
  void setMovementSpeed(short a_movement_speed);
  void setTime(short a_time);
  short pwmToDegrees ();

  short getChannel() const;

  private:
  short channel;
  short incoming_pwm;
  short movement_speed;
  short time;
  short current_degrees;
  
};