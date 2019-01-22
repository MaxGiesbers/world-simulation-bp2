#include "VirtualServo.hpp"

VirtualServo::VirtualServo() : channel(0), pwm(0), movement_speed(0), time(0)
{
}

VirtualServo::~VirtualServo()
{
}

void VirtualServo::setChannel(short a_channel)
{
  channel = a_channel;
}
void VirtualServo::setPwm(short a_pwm)
{
  pwm = a_pwm;
}
void VirtualServo::setMovementSpeed(short a_movement_speed)
{
  movement_speed = a_movement_speed;
}
void VirtualServo::setTime(short a_time)
{
  time = a_time;
}

