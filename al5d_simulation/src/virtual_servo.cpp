#include "virtual_servo.hpp"
#include <chrono>
#include <thread>

VirtualServo::VirtualServo(short a_channel) : channel(a_channel), movement_speed(0), time(0), current_degrees(0)
{
  servo_degrees_publisher = n.advertise<al5d_simulation::servo_position>("servo_position", 1000);
  msg_subscriber = n.subscribe("servo_command", 1000, &VirtualServo::publishMessage, this);  
}

VirtualServo::~VirtualServo()
{
}

void VirtualServo::publishMessage(const al5d_simulation::servo_command& servo)
{
  double degrees = 0;

  servo_position_msg.channel = servo.channel;

  if (channel == servo.channel)
  {
   

    if (servo.channel == 5 || servo.channel == 6)
    {
      degrees = mapGripper(servo.pwm);
    }
    else 
    {
      degrees = pwmToDegrees(servo.pwm);
    }
  
    while (!doubleEquals(degrees, current_degrees))
    {
      if (channel == 5 || channel == 6)
      {
        updateGripperPosition(current_degrees, degrees);
      }
      else
      {
        updateServoPositions(current_degrees, degrees);
      }

      servo_position_msg.degrees = current_degrees;
      servo_degrees_publisher.publish(servo_position_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}


bool VirtualServo::doubleEquals(double a, double b)
{
  const double EPSILON = 0.1;
  return std::abs(a - b) < EPSILON;
}

void VirtualServo::updateServoPositions(double& current_degrees, double degrees)
{
  if (current_degrees < degrees)
  {
    current_degrees++;
  }
  else if (current_degrees > degrees)
  {
    current_degrees--;
  }
}

void VirtualServo::updateGripperPosition(double& current_degrees, double degrees)
{
  if (current_degrees < degrees)
  {
    current_degrees = current_degrees + 0.01;
  }
  else if (current_degrees > degrees)
  {
    current_degrees = current_degrees - 0.01;
  }
}

double VirtualServo::pwmToDegrees(short incoming_pwm)
{
  const short max_pwm = 2500;
  const short min_degrees = -90;
  const short max_degrees = 90;
  const short min_pwm = 500;

  return (short)(((long double)(max_degrees - min_degrees) / (long double)(max_pwm - min_pwm)) *
                 (incoming_pwm - min_pwm)) +
         min_degrees;
}

double VirtualServo::mapGripper(short incoming_pwm)
{
  const short max_pwm = 2500;
  const double min_degrees = 0.0;
  const double max_degrees = 0.95;
  const short min_pwm = 500;

  return (double)(((long double)(max_degrees - min_degrees) / (long double)(max_pwm - min_pwm)) *
                 (incoming_pwm - min_pwm)) +
         min_degrees;
}

int main(int argc, char** argv)
{
  if (argc != 4)
  {
    ROS_ERROR("Not enough arguments are given , %d given", argc);
    return 1;
  }
  ros::init(argc, argv, "VirtualController" /*"Virtual servo" + std::stoi(argv[1])*/);
  
  VirtualServo servo(std::stoi(argv[1]));
  ros::spin();
}
