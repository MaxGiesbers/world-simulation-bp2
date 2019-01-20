
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <al5d_simulation/servo_command.h>

class CommandParser
{

  private:
  
  public:
    al5d_simulation::servo_command incomingCommand;

    CommandParser();
    ~CommandParser();

    void messageCallback(const std_msgs::String &msg);
    ros::Subscriber msg_subscriber;
    
    ros::NodeHandle n;
};