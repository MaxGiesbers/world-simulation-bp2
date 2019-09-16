#include "virtual_controller.hpp"
#include <thread>

namespace
{
  const uint8_t MINIMAL_ARGUMENTS = 6;
  const uint8_t ROS_LOOP_RATE = 20;
  const uint16_t QUEUE_SIZE = 1000; 
}

VirtualController::VirtualController(const Position& robot_arm_position) : m_command_character('\0')
{
  m_subscriber = m_node_handle.subscribe("msgPublisher", QUEUE_SIZE, &VirtualController::parseMessage, this);
  m_publisher = m_node_handle.advertise<al5d_simulation::servo_command>("servo_command", QUEUE_SIZE);
}

VirtualController::~VirtualController()
{
}

short VirtualController::getFirstIndexPosition(const std::string& message)
{
  short char_position = 0;
  for (const char delimeter : delimeters)
  {
    char_position = message.find(delimeter);

    if (char_position != std::string::npos && m_command_character != delimeter)
    {
      break;
    }
  }
  return char_position;
}

void VirtualController::setFirstAndSecondCharPosition(const std::string& message, uint8_t& first_position,
                                                      uint8_t& second_position)
{
  // reset command character
  m_command_character = '\0';

  first_position = getFirstIndexPosition(message);
  const std::size_t NUMBER_OF_LOOPS = 2;

  for (int i = 0; i < NUMBER_OF_LOOPS; i++)
  {
    for (const uint8_t delimeter : delimeters)
    {
      uint8_t tempPosition = message.find(delimeter);

      if (tempPosition != std::string::npos)
      {
        // Finds the lowest position of the first character
        if (tempPosition < first_position)
        {
          first_position = tempPosition;
        }
        // Finds the lowest position of the second character that is not the same as the first one.
        if (tempPosition < second_position && m_command_character != delimeter && m_command_character != '\0')
        {
          second_position = tempPosition;
        }
      }
    }
    // Breaks out of the function for keeping the second position
    if (m_command_character != '\0')
    {
      break;
    }
    m_command_character = message.at(first_position);
    second_position = getFirstIndexPosition(message);
  }
}

void VirtualController::parseMessage(const std_msgs::String& incoming_message)
{
  std::string message = incoming_message.data.c_str();
  remove_if(message.begin(), message.end(), isspace);

  uint8_t first_position = 0;
  uint8_t second_position = 0;

  std::string command = "";

  bool end_line = false;
  bool send_command = false;

  while (!end_line)
  {
    setFirstAndSecondCharPosition(message, first_position, second_position);
    // get command
    command = message.substr(first_position + 1, second_position - 1);
    message.erase(first_position, second_position);

    switch (m_command_character)
    {
      case '#':
      {
        if (send_command)
        {
          send_command = false;
        }
        m_servo_message.channel = stoi(command);
        send_command = true;
        break;
      }

      case 'P':
      {
        m_servo_message.pwm = stoi(command);
        break;
      }

      case 'S':
      {
        m_servo_message.speed = stoi(command);
        break;
      }

      case 'T':
      {
        m_servo_message.time = stoi(command);
        break;
      }
      case '\r':
      {
        ROS_INFO_STREAM("Publish Message #" << (int) m_servo_message.channel << "P" << m_servo_message.pwm << "S" << m_servo_message.speed  << "T" << m_servo_message.time);
        m_publisher.publish(m_servo_message);
        end_line = true;
        break;
      }
    }
  }
}

void runRobotArmSimulation(Position robot_arm_position)
{
  LynxMotionSimulator simulator(robot_arm_position);
  ros::Rate rate(ROS_LOOP_RATE);

  while (ros::ok())
  {
    simulator.publishStatesOfJoints();
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  if (argc != MINIMAL_ARGUMENTS)
  {
    ROS_ERROR("Not enough arguments are given , %d given", argc);
    return 1;
  }
  
  // initialize position robotarm
  Position robot_arm_position;
  robot_arm_position.x_pos = atof(argv[1]);
  robot_arm_position.y_pos = atof(argv[2]);
  robot_arm_position.z_pos = atof(argv[3]);

  ros::init(argc, argv, "VirtualController");
  std::thread robot_arm_simulation(runRobotArmSimulation, robot_arm_position);
  VirtualController p(robot_arm_position);

  ros::spin();
  robot_arm_simulation.join();

  return 0;
}
