#ifndef LYNX_MOTION_SIMULATOR_H
#define LYNX_MOTION_SIMULATOR_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "al5d_simulation/servo_position.h"
#include "position.h"
#include <mutex>

class LynxMotionSimulator
{
private:
  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;
  geometry_msgs::TransformStamped m_world_transform;
  tf::TransformBroadcaster m_broadcaster;
  sensor_msgs::JointState m_joint_state;
  ros::NodeHandle m_node_handle;
  Position m_robot_arm_position;
  std::mutex m_angle_mutex;

  void initializeJoints();
  void callBack(const al5d_simulation::servo_position& servo_position);

public:
  LynxMotionSimulator(const Position& a_robot_arm_position);
  ~LynxMotionSimulator();
  void publishStatesOfJoints();
};

#endif
