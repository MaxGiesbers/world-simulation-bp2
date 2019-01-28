#include "LynxMotionSimulator.hpp"

LynxMotionSimulator::LynxMotionSimulator(const RobotArmPosition& a_robot_arm_position)
  : degree(M_PI / 180), robot_arm_position(a_robot_arm_position)
{
  std::cout << "initialize joints " << std::endl;
  initializeJoints();
  servo_subscriber = n.subscribe("servo_degrees", 1000, &LynxMotionSimulator::publishCommands, this);
  joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  
}


void LynxMotionSimulator::initializeJoints()
{
  
  world_transform.header.frame_id = "world";
  world_transform.child_frame_id = "base_link";
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
  for(size_t i = 0; i < joint_state.position.size(); i++ )
  {
    joint_state.position[i] = 0;
  }
}

LynxMotionSimulator::~LynxMotionSimulator()
{
}

void LynxMotionSimulator::publishCommands(const al5d_simulation::servo_command& servo_degrees)
{
  joint_state.header.stamp = ros::Time::now();
  short degrees = servo_degrees.degrees;
  short channel = servo_degrees.channel;
  std::cout << "received command:" << degrees<< std::endl;
  std::cout << "servo channel: " << channel << std::endl;

  //set received channel en position in radians
  joint_state.position[channel] = degrees * M_PI / 180.0;

  world_transform.transform.translation.x = robot_arm_position.x_pos;
  world_transform.transform.translation.y = robot_arm_position.y_pos;
  world_transform.transform.translation.z = robot_arm_position.z_pos;
  
  using namespace std::chrono_literals;
  world_transform.transform.rotation = tf::createQuaternionMsgFromYaw(180 * degree);
  std::this_thread::sleep_for(75ms);
  // send the joint state and transform
  joint_publisher.publish(joint_state);
  broadcaster.sendTransform(world_transform);
}