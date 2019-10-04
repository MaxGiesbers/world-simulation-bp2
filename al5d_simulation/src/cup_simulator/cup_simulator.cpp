#include <tf/transform_listener.h>
#include "cup_simulator/cup.h"

namespace
{
const uint8_t MINIMAL_ARGUMENTS = 7;
}

int main(int argc, char** argv)
{
  if (argc != MINIMAL_ARGUMENTS)
  {
    ROS_ERROR("Not enough arguments are given, %d given", argc);
    return 1;
  }

  ros::init(argc, argv, "CupSimulator");

  Position cup_position;
  cup_position.x_pos = atof(argv[1]);
  cup_position.y_pos = atof(argv[2]);
  cup_position.z_pos = atof(argv[3]);
  Cup cup(argv[4], cup_position);
  cup.simulate();
  return 0;
}