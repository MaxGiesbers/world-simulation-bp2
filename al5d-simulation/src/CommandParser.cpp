#include "CommandParser.hpp"

CommandParser::CommandParser() : commandCharacter('\0')
{
  msg_subscriber = n.subscribe("msgPublisher", 1000, &CommandParser::parseMessage, this);
}

CommandParser::~CommandParser()
{
}

std::size_t CommandParser::findFirstArrayIndex(const std::string &message)
{

  std::size_t position = 0;
  for (const char delimeter : delimeters)
  {
    position = message.find(delimeter);

    if (position != std::string::npos && commandCharacter != delimeter)
    {
      break;
    }
  }
  return position;
}

void CommandParser::findCharacters(const std::string &message, size_t &firstPosition, size_t &secondPosition)
{
  //Get the first legit array position.
  commandCharacter = '\0';

  firstPosition = findFirstArrayIndex(message);
  const std::size_t numberOfLoops = 2;

  for (int i = 0; i < numberOfLoops; i++)
  {
    for (const char delimeter : delimeters)
    {
      std::size_t tempPosition = message.find(delimeter);

      if (tempPosition != std::string::npos)
      {
        //Finds the lowest position of the first character
        if (tempPosition < firstPosition)
        {
          firstPosition = tempPosition;
        }
        //Finds the lowest position of the second character that is not the same as the first one.
        if (tempPosition < secondPosition && commandCharacter != delimeter && commandCharacter != '\0')
        {
          secondPosition = tempPosition;
        }
      }
    }
    //Breaks out of the function for keeping the second position
    if (commandCharacter != '\0')
    {
      break;
    }
    commandCharacter = message.at(firstPosition);
    secondPosition = findFirstArrayIndex(message);
  }
}

void CommandParser::parseMessage(const std_msgs::String &msg)
{

  std::string message =  msg.data.c_str();

  remove_if(message.begin(), message.end(), isspace);

  size_t firstPosition = 0;
  size_t secondPosition = 0;

  std::string token = "";

  bool endOfLine = false;

  while (!endOfLine)
  {
    std::cout << "woord: " << message << std::endl;
    findCharacters(message, firstPosition, secondPosition);
    token = message.substr(firstPosition + 1, secondPosition - 1);
    std::cout << "command: " << token << std::endl;
    message.erase(firstPosition, secondPosition);

    switch (commandCharacter)
    {
    case '#':
    {
      std::cout << "# " << token << std::endl;

      break;
    }

    case 'P':
    {
      std::cout << "P " << token << std::endl;
      break;
    }

    case 'S':
    {
      std::cout << "S " << token << std::endl;
      break;
    }

    case 'T':
    {
      std::cout << "T " << token << std::endl;
      break;
    }
    case '\r':
    {
      endOfLine = true;
      break;
    }
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "CommandParser");
  ros::NodeHandle n;

  CommandParser p;

  ros::spin();
}
