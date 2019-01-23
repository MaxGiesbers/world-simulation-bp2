#include "VirtualServo.hpp"

VirtualServo::VirtualServo(short a_channel) : channel(a_channel), incoming_pwm(0), movement_speed(0), time(0), current_degrees(0)
{
}

VirtualServo::~VirtualServo()
{
}

void VirtualServo::setChannel(short a_channel)
{
  channel = a_channel;
}
void VirtualServo::setIncomingPwm(short a_incoming_pwm)
{
  incoming_pwm = a_incoming_pwm;
}
void VirtualServo::setMovementSpeed(short a_movement_speed)
{
  movement_speed = a_movement_speed;
}
void VirtualServo::setTime(short a_time)
{
  time = a_time;
}

short VirtualServo::pwmToDegrees()
{
  const short max_pwm = 2500;
  short min_degrees = -90;
  short max_degrees = 90;
  short min_pwm = 500;

  // If needed dont know atm.
  if (channel == 1)
  {
    min_degrees = -30;
    max_degrees = 90;
    min_pwm = 1100;
  }
  else if (channel == 2)
  {
    min_degrees = 0;
    max_degrees = 135;
    min_pwm = 650;
  }
  short current_degrees =
      (short)(((long double)(max_degrees - min_degrees) / (long double)(max_pwm - min_pwm)) * (incoming_pwm - min_pwm)) +
      min_degrees;
}
