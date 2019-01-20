#include "CommandParser.hpp"


CommandParser::CommandParser()
{
  msg_subscriber = n.subscribe("msgPublisher", 1000, &CommandParser::messageCallback, this);
}

CommandParser::~CommandParser()
{

}

void CommandParser::messageCallback(const std_msgs::String &msg)
{
  std::cout << msg.data.c_str() << std::endl;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "CommandParser");
    ros::NodeHandle n;
    
    CommandParser p;

    ros::spin();
}
