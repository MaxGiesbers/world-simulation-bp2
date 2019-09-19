#include <string>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"

namespace
{
const double LOOP_RATE = 0.75;
const uint8_t QUEUE_SIZE = 1;
const uint8_t MINIMAL_ARGUMENTS = 5;
const uint8_t START_UP_TIME = 5;
}  // namespace

int main(int argc, char** argv)
{
  std::cout << argv[1] << std::endl;
  std::ifstream myfile(argv[2]);
  if (argc != MINIMAL_ARGUMENTS)
  {
    ROS_ERROR("Not enough arguments are dfdfdffd given, %d given", argc);
    return 1;
  }

  if (myfile.fail())
  {
    ROS_ERROR("Path or file unreadable ");
    return 1;
  }

  ros::init(argc, argv, "publisher");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(LOOP_RATE);
  std_msgs::String commandMsg;
  ros::Publisher msg_publisher = node_handle.advertise<std_msgs::String>("msgPublisher", QUEUE_SIZE);

  ROS_INFO("Robot arm simulation demo started");
  ros::Duration(START_UP_TIME).sleep();

  std::string line = "";
  if (myfile.is_open())
  {
    while ((myfile.good()) && !(myfile.peek() == std::ifstream::traits_type::eof()))
    {
      // Check for subscribers
      if (msg_publisher.getNumSubscribers() > 0)
      {
        getline(myfile, line);
        if (!line.empty())
        {
          commandMsg.data = line;
          commandMsg.data += '\r';
          ROS_INFO("Sending to the controller command: %s ", commandMsg.data.c_str());
          msg_publisher.publish(commandMsg);
        }
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    myfile.close();
  }
  else
  {
    ROS_ERROR("Unable to open file");
  }

  ROS_INFO("Robot arm simulation demo finished");
  return 0;
}