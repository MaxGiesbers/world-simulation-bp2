#include <string>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    std::ifstream myfile(argv[1]);

    if (argc != 2)
    {
        ROS_ERROR("Not enough arguments are given, %d given", argc);
        return 1;
    }

    if (myfile.fail())
    {
        ROS_ERROR("Path or file unreadable ");
        return -1;
    }

    ros::init(argc, argv, "Publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);
    std_msgs::String commandMsg;
    ros::Publisher msg_publisher = n.advertise<std_msgs::String>("msgPublisher", 1000);

    std::string line = "";
    if (myfile.is_open())
    {

        while (myfile.good())
        {
            //Check for subscribers
            if (msg_publisher.getNumSubscribers() > 0)
            {
                getline(myfile, line);
                commandMsg.data = line;
                commandMsg.data +='\r';
                ROS_INFO("Sending to the controller command: %s ", commandMsg.data.c_str());
                msg_publisher.publish(commandMsg);
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
    return 0;
}