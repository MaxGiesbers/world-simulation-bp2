#ifndef CUP_H
#define CUP_H

#include <ros/ros.h>
#include "al5d_simulator/position.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

enum class State
{
  Released,
  Grabbed,
  Dropping
};

class Cup
{
private:
  ros::Publisher m_publisher;
  std::string m_cup_name;
  ros::NodeHandle m_node_handle;
  Position m_cup_position;

  visualization_msgs::Marker m_cup_marker;
  geometry_msgs::TransformStamped m_world_transform;
  tf::TransformBroadcaster m_broadcaster;
  tf::StampedTransform m_transform;
  tf::TransformListener m_listener;
  State m_cup_state;
  void updateCupPosition(tf::StampedTransform& tf_grippper_right_cup, tf::StampedTransform& tf_grippper_left_cup,
                         tf::StampedTransform& tf_world_cup);
  void publishCupStatus();
  void updateCupColor();
  void initializeCup();
  void initializeRosWorld();
  bool cupGrapped(const tf::StampedTransform& tf_grippper_right_cup, const tf::StampedTransform& tf_grippper_left_cup);

public:
  Cup(std::string cup_name, Position cup_position);
  ~Cup();
  void simulate();
};

#endif