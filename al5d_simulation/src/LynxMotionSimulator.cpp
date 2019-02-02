#include "LynxMotionSimulator.hpp"

LynxMotionSimulator::LynxMotionSimulator(const Position& a_robot_arm_position)
  : degree(M_PI / 180), robot_arm_position(a_robot_arm_position), current_degrees(0), current_channel(0)
{
  initializeJoints();
  servo_subscriber = n.subscribe("servo_degrees", 1000, &LynxMotionSimulator::callBack, this);
  joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void LynxMotionSimulator::initializeJoints()
{
  world_transform.header.frame_id = "world";
  world_transform.child_frame_id = "base_link";
  world_transform.transform.translation.x = robot_arm_position.x_pos;
  world_transform.transform.translation.y = robot_arm_position.y_pos;
  world_transform.transform.translation.z = robot_arm_position.z_pos;

  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(7);
  joint_state.position.resize(7);
  joint_state.name[0] = "base_link2turret";
  joint_state.name[1] = "turret2upperarm";
  joint_state.name[2] = "upperarm2forearm";
  joint_state.name[3] = "forearm2wrist";
  joint_state.name[4] = "wrist2hand";
  joint_state.name[5] = "gripper_left2hand";
  joint_state.name[6] = "gripper_right2hand";
  for (size_t i = 0; i < joint_state.position.size(); i++)
  {
    joint_state.position[i] = 0;
  }
}

void LynxMotionSimulator::publishStatesOfJoints()
{
  joint_state.header.stamp = ros::Time::now();
  world_transform.header.stamp = ros::Time::now();
  world_transform.transform.rotation = tf::createQuaternionMsgFromYaw(180 * degree);

  joint_publisher.publish(joint_state);
  broadcaster.sendTransform(world_transform);
}

LynxMotionSimulator::~LynxMotionSimulator()
{
}

void LynxMotionSimulator::callBack(const al5d_simulation::servo_command& servo_degrees)
{
  std::lock_guard<std::mutex> guard(angle_mutex_);
  joint_state.header.stamp = ros::Time::now();
  current_degrees = servo_degrees.degrees;
  current_channel = servo_degrees.channel;

 

  // set received channel en position in radians
  joint_state.position[current_channel] = current_degrees * M_PI / 180.0;

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(20ms);
}