#include "al5d_simulator/virtual_servo.hpp"
#include <thread>

namespace
{
const uint8_t GRIPPER_LEFT = 5;
const uint8_t GRIPPER_RIGHT = 6;
const uint8_t MINIMAL_ARGUMENTS = 4;
const uint16_t MAX_PWM = 2500;
const uint16_t MIN_PWM = 500;
const uint16_t THREAD_SLEEP_DURATION = 10;
const uint16_t SERVO_MOVEMENT_SPEED = 1;
const double GRIPPER_MOVEMENT_SPEED = 0.01;
const double EPSILON = 0.1;
}  // namespace

VirtualServo::VirtualServo(uint8_t channel) : m_channel(channel), m_movement_speed(0), m_time(0), m_current_degrees(0)
{
  m_publisher = m_node_handle.advertise<al5d_simulation::servo_position>("servo_position", 1000);
  m_subscriber = m_node_handle.subscribe("servo_command", 1000, &VirtualServo::publishMessage, this);
}

VirtualServo::~VirtualServo()
{
}

void VirtualServo::publishMessage(const al5d_simulation::servo_command& servo)
{
  double incoming_degrees = 0;

  m_servo_message.channel = servo.channel;

  if (m_channel == servo.channel)
  {
    if (servo.channel == GRIPPER_LEFT || servo.channel == GRIPPER_RIGHT)
    {
      incoming_degrees = mapGripper(servo.pwm);
    }
    else
    {
      incoming_degrees = mapServo(servo.pwm);
    }

    while (!almostEquals(incoming_degrees, m_current_degrees))
    {
      if (m_channel == GRIPPER_LEFT || m_channel == GRIPPER_RIGHT)
      {
        updateServoPosition(incoming_degrees, GRIPPER_MOVEMENT_SPEED);
      }
      else
      {
        updateServoPosition(incoming_degrees, SERVO_MOVEMENT_SPEED);
      }

      m_servo_message.degrees = m_current_degrees;
      m_publisher.publish(m_servo_message);
      std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_DURATION));
    }
  }
}

bool VirtualServo::almostEquals(double a, double b)
{
  return std::abs(a - b) < EPSILON;
}

void VirtualServo::updateServoPosition(double incoming_degrees, double movement_speed)
{
  if (m_current_degrees < incoming_degrees)
  {
    m_current_degrees = m_current_degrees + movement_speed;
  }
  else if (m_current_degrees > incoming_degrees)
  {
    m_current_degrees = m_current_degrees - movement_speed;
  }
}

double VirtualServo::mapServo(uint16_t incoming_pwm)
{
  const int16_t MIN_DEGREES = -90;
  const int16_t MAX_DEGREES = 90;

  return (((double)(MAX_DEGREES - MIN_DEGREES) / (double)(MAX_PWM - MIN_PWM)) * (incoming_pwm - MIN_PWM)) + MIN_DEGREES;
}

double VirtualServo::mapGripper(uint16_t incoming_pwm)
{
  const double MIN_DEGREES = 0.0;
  const double MAX_DEGREES = 0.95;

  return (((double)(MAX_DEGREES - MIN_DEGREES) / (double)(MAX_PWM - MIN_PWM)) * (incoming_pwm - MIN_PWM)) + MIN_DEGREES;
}

int main(int argc, char** argv)
{
  if (argc != MINIMAL_ARGUMENTS)
  {
    ROS_ERROR("Not enough arguments are given , %d given", argc);
    return 1;
  }

  ros::init(argc, argv, "Virtual Servo" + std::stoi(argv[1]));
  VirtualServo servo(std::stoi(argv[1]));
  ros::spin();
}
