#include <ros/ros.h>
#include "position.hpp"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

enum class State
{
  Released,
  Grabbed,
  Dropped
};

class Cup
{
private:
  ros::Publisher cup_publisher;
  std::string cup_name;
  ros::NodeHandle n;
  Position cup_position;

  visualization_msgs::Marker cup_marker;
  geometry_msgs::TransformStamped world_transform;
  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  State cup_state;
  bool grapped;
  void updateCupState(State cup_state, tf::StampedTransform& tf_grippper_right_cup, tf::StampedTransform& tf_grippper_left_cup, tf::StampedTransform& tf_world_cup);

public:
  Cup(std::string a_cup_name, Position a_cup_position);
  ~Cup();
  void publishStatus();
  void simulate();
};
