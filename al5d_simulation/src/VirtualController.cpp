#include "VirtualController.hpp"

VirtualController::VirtualController(const RobotArmPosition& robot_arm_position)
  : command_character('\0'), simulator(robot_arm_position)
{
  initServoList();
  msg_subscriber = n.subscribe("msgPublisher", 1000, &VirtualController::parseMessage, this);
  servo_degrees_publisher = n.advertise<al5d_simulation::servo_command>("servo_degrees", 1000);
}

VirtualController::~VirtualController()
{
}

void VirtualController::publishTestMessage()
{
  servo_degrees_msg.degrees = 90;
  // 725
  double value = -(1.0 / 2.0) * std::acos((double)-1);
  //-1.5708

  // std::cout << value << std::endl;
  servo_degrees_msg.channel = 0;
  std::cout << "publish message" << std::endl;
  std::cout << servo_degrees_msg.degrees << std::endl;
  servo_degrees_publisher.publish(servo_degrees_msg);
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
        std::cout << "# " << command << std::endl;
        current_servo = getMatchingServo(stoi(command));
        break;
      }

      case 'P':
      {
        std::cout << "P " << command << std::endl;
        current_servo.setIncomingPwm(stoi(command));
        break;
      }

      case 'S':
      {
        std::cout << "S " << command << std::endl;
        current_servo.setMovementSpeed(stoi(command));
        break;
      }

      case 'T':
      {
        std::cout << "T " << command << std::endl;
        current_servo.setTime(stoi(command));
        break;
      }
      case '\r':
      {
        current_servo.publishMessage();
        end_line = true;
        break;
      }
    }
  }
}

void VirtualController::initServoList()
{
  const short number_of_servos = 6;
  for (size_t i = 0; i < 6; i++)
  {
    servo_list.push_back(VirtualServo(i));
  }
}

const VirtualServo& VirtualController::getMatchingServo(const short incoming_channel)
{
  // check inbouwen
  auto servo = find_if(servo_list.begin(), servo_list.end(),
                       [incoming_channel](VirtualServo& s) { return s.getChannel() == incoming_channel; });

  std::cout << servo->getChannel() << std::endl;
  return *servo;
}

int main(int argc, char** argv)
{
  if (argc != 6)
  {
    ROS_ERROR("Not enough arguments are given blablall, %d given", argc);
    return 1;
  }

  // initialize position robotarm
  RobotArmPosition position;
  std::cout << argv[1] << std::endl;
  position.x_pos = atof(argv[1]);
  position.y_pos = atof(argv[2]);
  position.z_pos = atof(argv[3]);

  ros::init(argc, argv, "VirtualController");
  ros::NodeHandle n;
  VirtualController p(position);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // // Check for subscribers
    // if (p.servo_degrees_publisher.getNumSubscribers() > 0)
    // {
    //   std::cout << "subscriber gevonden " << std::endl;
    //   //p.publishTestMessage();
    //   break;
    // }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}
