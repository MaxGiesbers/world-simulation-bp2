#include "virtual_controller.hpp"
#include <thread>

VirtualController::VirtualController(const Position& robot_arm_position) : command_character('\0')
{
  msg_subscriber = n.subscribe("msgPublisher", 1000, &VirtualController::parseMessage, this);
  servo_publisher = n.advertise<al5d_simulation::servo_command>("servo_command", 1000);
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

    if (char_position != std::string::npos && command_character != delimeter)
    {
      break;
    }
  }
  return char_position;
}

void VirtualController::setFirstAndSecondCharPosition(const std::string& message, short& first_position,
                                                      short& second_position)
{
  // reset command character
  command_character = '\0';

  first_position = getFirstIndexPosition(message);
  const std::size_t numberOfLoops = 2;

  for (int i = 0; i < numberOfLoops; i++)
  {
    for (const char delimeter : delimeters)
    {
      char tempPosition = message.find(delimeter);

      if (tempPosition != std::string::npos)
      {
        // Finds the lowest position of the first character
        if (tempPosition < first_position)
        {
          first_position = tempPosition;
        }
        // Finds the lowest position of the second character that is not the same as the first one.
        if (tempPosition < second_position && command_character != delimeter && command_character != '\0')
        {
          second_position = tempPosition;
        }
      }
    }
    // Breaks out of the function for keeping the second position
    if (command_character != '\0')
    {
      break;
    }
    command_character = message.at(first_position);
    second_position = getFirstIndexPosition(message);
  }
}

void VirtualController::parseMessage(const std_msgs::String& msg)
{
  std::string message = msg.data.c_str();
  remove_if(message.begin(), message.end(), isspace);

  short first_position = 0;
  short second_position = 0;

  std::string command = "";

  bool end_line = false;
  bool send_command = false;


  //VirtualServo current_servo;

  while (!end_line)
  {
    setFirstAndSecondCharPosition(message, first_position, second_position);
    // get command
    command = message.substr(first_position + 1, second_position - 1);
    message.erase(first_position, second_position);

    switch (command_character)
    {
      case '#':
      {
        if (send_command)
        {
          //??????
          servo_publisher.publish(servo_degrees_msg);
          send_command = false;
        }
        std::cout << "# " << command << std::endl;
        servo_degrees_msg.channel = stoi(command);
        send_command = true;

        break;
      }

      case 'P':
      {
        std::cout << "P " << command << std::endl;
        servo_degrees_msg.pwm = stoi(command);
        //current_servo.setIncomingPwm(stoi(command));
        break;
      }

      case 'S':
      {
        std::cout << "S " << command << std::endl;
        servo_degrees_msg.speed = stoi(command);
        break;
      }

      case 'T':
      {
        std::cout << "T " << command << std::endl;
        servo_degrees_msg.time = stoi(command);
        break;
      }
      case '\r':
      {
        // publih
        servo_publisher.publish(servo_degrees_msg);
        std::cout << "publish /r " << std::endl;
        end_line = true;
        break;
      }
    }
  }
}

void runRobotArmSimulation(Position robot_arm_position)
{
  LynxMotionSimulator simulator(robot_arm_position);
  ros::Rate rate(10);

  while (ros::ok())
  {
    simulator.publishStatesOfJoints();
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  if (argc != 6)
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
  ros::NodeHandle n;
  std::thread robot_arm_simulation(runRobotArmSimulation, robot_arm_position);
  VirtualController p(robot_arm_position);

  ros::spin();
  robot_arm_simulation.join();

  return 0;
}
