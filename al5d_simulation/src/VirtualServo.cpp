#include "VirtualServo.hpp"

VirtualServo::VirtualServo(short a_channel)
  : channel(a_channel), incoming_pwm(0), movement_speed(0), time(0), current_degrees(0)
{
  servo_degrees_publisher = n.advertise<al5d_simulation::servo_command>("servo_degrees", 1000);
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

short VirtualServo::getChannel() const
{
  return channel;
}

void VirtualServo::publishMessage()
{
  // Convert received pwm to degrees
  short degrees = pwmToDegrees();
  std::cout << "publish message " << std::endl;
  std::cout << channel << std::endl;
  std::cout << incoming_pwm << std::endl;
  std::cout << movement_speed << std::endl;
  std::cout << time << std::endl;
  std::cout << degrees << std::endl;

  //ophogen voor nu
  while (degrees != current_degrees)
  {
    if (degrees < current_degrees)
    {
      degrees++;
    }
    else if (degrees > current_degrees)
    {
      degrees--;
    }
    servo_degrees_msg.degrees = degrees;
    servo_degrees_msg.channel = channel;
    servo_degrees_publisher.publish(servo_degrees_msg);
  }
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