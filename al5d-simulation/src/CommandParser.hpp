
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <al5d_simulation/servo_command.h>
#include <vector>

class CommandParser
{

  private:
    char commandCharacter;
  
  public:
    al5d_simulation::servo_command incomingCommand;

    std::vector <char> delimeters = {'#', 'P', 'S', 'T', '\r'}; 

    CommandParser();
    ~CommandParser();

    void parseMessage(const std_msgs::String &msg);
    ros::Subscriber msg_subscriber;

    void findCharacters (const std::string& message, size_t& firstPosition, size_t& secondPosition);
    std::size_t findFirstArrayIndex(const std::string &message);

    ros::NodeHandle n;
};