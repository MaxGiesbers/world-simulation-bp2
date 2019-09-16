#include "lynx_motion_simulator.hpp"
#include <thread>

namespace 
{
  const uint8_t THREAD_SLEEP_DURATION = 30;
  const uint16_t QUEUE_SIZE = 1000; 
  const double DEGREES = M_PI / 180;
  const uint8_t ARRAY_SIZE = 7;
}

LynxMotionSimulator::LynxMotionSimulator(const Position& a_robot_arm_position)
  : m_robot_arm_position(a_robot_arm_position)
{
  initializeJoints();
  m_subscriber = m_node_handle.subscribe("servo_position", QUEUE_SIZE, &LynxMotionSimulator::callBack, this);
  m_publisher = m_node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void LynxMotionSimulator::initializeJoints()
{
  m_world_transform.header.frame_id = "world";
  m_world_transform.child_frame_id = "base_link";
  m_world_transform.transform.translation.x = m_robot_arm_position.x_pos;
  m_world_transform.transform.translation.y = m_robot_arm_position.y_pos;
  m_world_transform.transform.translation.z = m_robot_arm_position.z_pos;

  m_joint_state.header.stamp = ros::Time::now();
  m_joint_state.name.resize(ARRAY_SIZE);
  m_joint_state.position.resize(ARRAY_SIZE);
  m_joint_state.name[0] = "base_link2turret";
  m_joint_state.name[1] = "turret2upperarm";
  m_joint_state.name[2] = "upperarm2forearm";
  m_joint_state.name[3] = "forearm2wrist";
  m_joint_state.name[4] = "wrist2hand";
  m_joint_state.name[5] = "gripper_left2hand";
  m_joint_state.name[6] = "gripper_right2hand";
  for (size_t i = 0; i < m_joint_state.position.size(); i++)
  {
    m_joint_state.position[i] = 0;
  }
  //publishStatesOfJoints();
}

void LynxMotionSimulator::publishStatesOfJoints()
{
  m_joint_state.header.stamp = ros::Time::now();
  m_world_transform.header.stamp = ros::Time::now();
  m_world_transform.transform.rotation = tf::createQuaternionMsgFromYaw(180 * DEGREES);

  m_publisher.publish(m_joint_state);
  m_broadcaster.sendTransform(m_world_transform);
}

LynxMotionSimulator::~LynxMotionSimulator()
{
}

void LynxMotionSimulator::callBack(const al5d_simulation::servo_position& servo_position)
{
  
  std::lock_guard<std::mutex> guard(m_angle_mutex);
  m_joint_state.header.stamp = ros::Time::now();

  // set received channel en position in radians
  m_joint_state.position[servo_position.channel] = servo_position.degrees * DEGREES;

  std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_DURATION));
}