#include <string>
#include <ros/ros.h>
#include<iostream>
#include<fstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "Publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);
    std::ifstream myfile (argv[1]);

    if(argc != 2)
    {
        ROS_ERROR("Not enough arguments are given, %d given", argc);
        return 1;
    }

    if(myfile.fail())
    {
        ROS_ERROR("Path or file unreadable ");
        return 1;
    }

    std::string line;

    if (myfile.is_open())
    {
        while ( myfile.good() )
        {
            getline (myfile,line);
            std::cout << line << std::endl;
        }
        myfile.close();
    }

    else std::cout << "Unable to open file"; 

    return 0;
}