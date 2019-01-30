#include <tf/transform_listener.h>
#include "Cup.hpp"

int main(int argc, char** argv)
{
  if (argc != 7)
  {
    ROS_ERROR("Not enough arguments are given, %d given", argc);
    return 1;
  }
  
  ros::init(argc, argv, "CupSimulation");

  Position cup_position;
  cup_position.x_pos = atof(argv[1]);
  cup_position.y_pos = atof(argv[2]);
  cup_position.z_pos = atof(argv[3]);
  Cup cup(argv[4], cup_position);
  cup.simulate();
  return 0;
}