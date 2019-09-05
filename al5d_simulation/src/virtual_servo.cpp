#include "virtual_servo.hpp"

VirtualServo::VirtualServo() : channel(0), incoming_pwm(0), movement_speed(0), time(0), current_degrees(0)
{
}

VirtualServo::~VirtualServo()
{
}

VirtualServo::VirtualServo(const VirtualServo& a_servo)
  : channel(a_servo.channel)
  , incoming_pwm(a_servo.incoming_pwm)
  , movement_speed(a_servo.movement_speed)
  , time(a_servo.time)
  , current_degrees(a_servo.current_degrees)
{
}

short VirtualServo::getIncomingPWM() const
{
  return incoming_pwm;
}
void VirtualServo::setChannel(short a_channel)
{
  channel = a_channel;
}
void VirtualServo::setIncomingPwm(const short a_incoming_pwm)
{
  incoming_pwm = a_incoming_pwm;
}
void VirtualServo::setMovementSpeed(const short a_movement_speed)
{
  movement_speed = a_movement_speed;
}
void VirtualServo::setTime(const short a_time)
{
  time = a_time;
}

short VirtualServo::getChannel() const
{
  return channel;
}

void VirtualServo::setCurrentDegrees(const short a_current_degrees)
{
  current_degrees = a_current_degrees;
}

short VirtualServo::getCurrentDegrees() const
{
  return current_degrees;
}

short VirtualServo::pwmToDegrees()
{
  const short max_pwm = 2500;
  const short min_degrees = -90;
  const short max_degrees = 90;
  const short min_pwm = 500;

  return (short)(((long double)(max_degrees - min_degrees) / (long double)(max_pwm - min_pwm)) *
                 (incoming_pwm - min_pwm)) +
         min_degrees;
}
