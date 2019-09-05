#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "al5d_simulation/servo_command.h"
#include "position.hpp"

#include <thread>
#include <mutex>

class LynxMotionSimulator
{
public:
  LynxMotionSimulator(const Position& a_robot_arm_position);
  ~LynxMotionSimulator();
  void callBack(const al5d_simulation::servo_command& servo_degrees);
  void publishStatesOfJoints();
  ros::Subscriber servo_subscriber;
  ros::Publisher joint_publisher;

private:
  void initializeJoints();
  geometry_msgs::TransformStamped world_transform;
  tf::TransformBroadcaster broadcaster;
  sensor_msgs::JointState joint_state;
  const double degree;
  Position robot_arm_position;
  std::mutex angle_mutex_;
  short current_degrees;
  short current_channel;

  ros::NodeHandle n;
};