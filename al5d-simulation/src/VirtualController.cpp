#include "VirtualController.hpp"

VirtualController::VirtualController() : command_character('\0')
{
  msg_subscriber = n.subscribe("msgPublisher", 1000, &VirtualController::parseMessage, this);
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

  while (!end_line)
  {
    setFirstAndSecondCharPosition(message, first_position, second_position);
    command = message.substr(first_position + 1, second_position - 1);
    message.erase(first_position, second_position);

    switch (command_character)
    {
      case '#':
      {
        std::cout << "# " << command << std::endl;
        servo.setChannel(stoi(command));
        break;
      }

      case 'P':
      {
        std::cout << "P " << command << std::endl;
        servo.setPwm(stoi(command));
        break;
      }

      case 'S':
      {
        std::cout << "S " << command << std::endl;
        servo.setMovementSpeed(stoi(command));
        break;
      }

      case 'T':
      {
        std::cout << "T " << command << std::endl;
        servo.setTime(stoi(command));
        break;
      }
      case '\r':
      {
        end_line = true;
        break;
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "VirtualController");
  ros::NodeHandle n;

  VirtualController p;

  ros::spin();
}
