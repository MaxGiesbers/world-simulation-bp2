#include "CommandParser.hpp"


CommandParser::CommandParser()
{

}

CommandParser::~CommandParser()
{

}

void CommandParser::messageCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "CommandParser");
    CommandParser test;

    test.incomingCommand.channel = 10;

    int testen = test.incomingCommand.channel;
    std::cout << testen << std::endl;
}
