#include "cup_simulator/cup.hpp"

namespace
{
  const double CUP_DROP_SPEED = 0.01;
  const uint8_t ROS_LOOP_RATE = 20;
  const double CUP_X_SCALE = 0.02;
  const double CUP_Y_SCALE = 0.02;
  const double CUP_Z_SCALE = 0.04;
  const double TRANSFORM_TIME_OUT = 0.5;
  const double Y_MINIMUM_MARGIN = 0.013;
  const double Y_MAXIMUM_MARGIN = 0.023;
  const double X_MINIMUM_MARGIN = 0.005;
  const double X_MAXIMUM_MARGIN = 0.018;
  const double Z_MINIMUM_MARGIN = 0.002;
  const double Z_MAXIMUM_MARGIN = 0.017;
}

Cup::Cup(std::string cup_name, Position cup_position) : m_cup_name(cup_name), m_cup_position(cup_position)
{

  initializeCup();
  initializeRosWorld();
  m_publisher = m_node_handle.advertise<visualization_msgs::Marker>(cup_name, 1);
  m_publisher.publish(m_cup_marker);
  m_broadcaster.sendTransform(m_world_transform);
}
Cup::~Cup()
{
}

void Cup::initializeRosWorld()
{
  m_world_transform.header.frame_id = "world";
  m_world_transform.child_frame_id = m_cup_name;
  m_world_transform.header.stamp = ros::Time::now();
  m_world_transform.transform.translation.x = m_cup_position.x_pos;
  m_world_transform.transform.translation.y = m_cup_position.y_pos;
  m_world_transform.transform.translation.z = m_cup_position.z_pos;
  m_world_transform.transform.rotation = tf::createQuaternionMsgFromYaw(0);
}
void Cup::initializeCup()
{
  m_cup_marker.ns = m_cup_name;
  m_cup_marker.id = 0;
  m_cup_marker.type = visualization_msgs::Marker::CYLINDER;
  m_cup_marker.action = visualization_msgs::Marker::ADD;
  m_cup_marker.scale.x = CUP_X_SCALE;
  m_cup_marker.scale.y = CUP_Y_SCALE;
  m_cup_marker.scale.z = CUP_Z_SCALE;
  m_cup_marker.header.frame_id = "world";
  m_cup_marker.pose.position.x = m_cup_position.x_pos;
  m_cup_marker.pose.position.y = m_cup_position.y_pos;
  m_cup_marker.pose.position.z = m_cup_position.z_pos;
  m_cup_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  m_cup_marker.lifetime = ros::Duration();
}

void Cup::simulate()
{
  ros::Rate loop_rate(ROS_LOOP_RATE);

  while (ros::ok())
  {
    m_world_transform.header.stamp = ros::Time::now();
    m_cup_marker.header.stamp = ros::Time::now();
    m_broadcaster.sendTransform(m_world_transform);
    publishCupStatus();

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Cup::updateCupColor()
{
  switch(m_cup_state)
  {
    case State::Grabbed:
    {
      m_cup_marker.color.a = 1.0;
      m_cup_marker.color.r = 0;
      m_cup_marker.color.g = 255;
      m_cup_marker.color.b = 0;
      break;
    }
    case State::Dropping:
    {
      m_cup_marker.color.a = 1.0;  
      m_cup_marker.color.r = 255;
      m_cup_marker.color.g = 0;
      m_cup_marker.color.b = 0;
      break;
    } 
    case State::Released:
    {
      m_cup_marker.color.a = 1.0;  
      m_cup_marker.color.r = 255;
      m_cup_marker.color.g = 255;
      m_cup_marker.color.b = 255;
      break;
    }
  }
}

bool Cup::cupGrapped(const tf::StampedTransform& tf_grippper_right_cup, const tf::StampedTransform& tf_grippper_left_cup)
{
  double diff_right_x = tf_grippper_right_cup.getOrigin().x();
  double diff_right_y = tf_grippper_right_cup.getOrigin().y();
  double diff_right_z = tf_grippper_right_cup.getOrigin().z();

  double diff_left_x = tf_grippper_left_cup.getOrigin().x() ;
  double diff_left_y = tf_grippper_left_cup.getOrigin().y() ;
  double diff_left_z = tf_grippper_left_cup.getOrigin().z() ;
   
  return (std::abs(diff_right_y) < Y_MAXIMUM_MARGIN && std::abs(diff_right_y) > Y_MINIMUM_MARGIN
      && std::abs(diff_left_y) < Y_MAXIMUM_MARGIN && std::abs(diff_left_y) > Y_MINIMUM_MARGIN
      && std::abs(diff_right_x) < X_MAXIMUM_MARGIN && std::abs(diff_right_x) > X_MINIMUM_MARGIN
      && std::abs(diff_left_x) < X_MAXIMUM_MARGIN && std::abs(diff_left_x) > X_MINIMUM_MARGIN
      && std::abs(diff_left_z) < Z_MAXIMUM_MARGIN && std::abs(diff_left_z) > Z_MINIMUM_MARGIN
      && std::abs(diff_right_z) < Z_MAXIMUM_MARGIN && std::abs(diff_right_z) > Z_MINIMUM_MARGIN);
}

void Cup::publishCupStatus()
{
  tf::StampedTransform tf_world_cup;
  tf::StampedTransform tf_gripper_right_cup;
  tf::StampedTransform tf_gripper_left_cup;
  try
  {
    m_listener.waitForTransform("world", m_cup_name, ros::Time(0), ros::Duration(TRANSFORM_TIME_OUT));
    m_listener.waitForTransform("gripper_right", m_cup_name, ros::Time(0), ros::Duration(TRANSFORM_TIME_OUT));
    m_listener.waitForTransform("gripper_left", m_cup_name, ros::Time(0), ros::Duration(TRANSFORM_TIME_OUT));
    
    m_listener.lookupTransform("world", m_cup_name, ros::Time(0), tf_world_cup);

    if (m_listener.canTransform("gripper_right", m_cup_name, ros::Time(0)) && m_listener.canTransform("gripper_left", m_cup_name, ros::Time(0)))
    {
      m_listener.lookupTransform("gripper_right", m_cup_name, ros::Time(0), tf_gripper_right_cup);
      m_listener.lookupTransform("gripper_left", m_cup_name, ros::Time(0), tf_gripper_left_cup);
    }

    if (cupGrapped(tf_gripper_right_cup,tf_gripper_left_cup))
    {
      m_cup_state = State::Grabbed;
    }
    else if (m_cup_position.z_pos < tf_world_cup.getOrigin().z())
    {    
      m_cup_state = State::Dropping;
    }
    else 
    {
      m_cup_state = State::Released;
    }

    updateCupPosition(tf_gripper_right_cup, tf_gripper_left_cup, tf_world_cup);
    updateCupColor();
    m_publisher.publish(m_cup_marker);

  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void Cup::updateCupPosition(tf::StampedTransform& tf_grippper_right_cup, tf::StampedTransform& tf_grippper_left_cup, tf::StampedTransform& tf_world_cup )
{
  switch(m_cup_state)
  {
    case State::Grabbed:
    { 
      m_world_transform.header.frame_id = "gripper_right";
      m_world_transform.transform.translation.x = tf_grippper_right_cup.getOrigin().x();
      m_world_transform.transform.translation.y = tf_grippper_right_cup.getOrigin().y();
      m_world_transform.transform.translation.z = tf_grippper_right_cup.getOrigin().z();
      m_world_transform.transform.rotation.x = tf_grippper_right_cup.getRotation().x();
      m_world_transform.transform.rotation.y = tf_grippper_right_cup.getRotation().y();
      m_world_transform.transform.rotation.z = tf_grippper_right_cup.getRotation().z();
      m_world_transform.transform.rotation.w = tf_grippper_right_cup.getRotation().w();

      m_cup_marker.header.frame_id = "gripper_right";
      m_cup_marker.pose.position.x = tf_grippper_right_cup.getOrigin().x();
      m_cup_marker.pose.position.y = tf_grippper_right_cup.getOrigin().y();
      m_cup_marker.pose.position.z = tf_grippper_right_cup.getOrigin().z();
      m_cup_marker.pose.orientation.x = tf_grippper_right_cup.getRotation().x();
      m_cup_marker.pose.orientation.y = tf_grippper_right_cup.getRotation().y();
      m_cup_marker.pose.orientation.z = tf_grippper_right_cup.getRotation().z();
      m_cup_marker.pose.orientation.w = tf_grippper_right_cup.getRotation().w();

      break;
    }
    case State::Released:
    {
      m_world_transform.header.frame_id = "world";
      m_world_transform.transform.translation.x = tf_world_cup.getOrigin().x();
      m_world_transform.transform.translation.y = tf_world_cup.getOrigin().y();
      m_world_transform.transform.translation.z = tf_world_cup.getOrigin().z();
      m_world_transform.transform.rotation.x = tf_world_cup.getRotation().x();
      m_world_transform.transform.rotation.y = tf_world_cup.getRotation().y();
      m_world_transform.transform.rotation.z = tf_world_cup.getRotation().z();
      m_world_transform.transform.rotation.w = tf_world_cup.getRotation().w();

      m_cup_marker.header.frame_id = "world";
      m_cup_marker.pose.position.x = tf_world_cup.getOrigin().x();
      m_cup_marker.pose.position.y = tf_world_cup.getOrigin().y();
      m_cup_marker.pose.position.z = tf_world_cup.getOrigin().z();
      m_cup_marker.pose.orientation.x = tf_world_cup.getRotation().x();
      m_cup_marker.pose.orientation.y = tf_world_cup.getRotation().y();
      m_cup_marker.pose.orientation.z = tf_world_cup.getRotation().z();
      m_cup_marker.pose.orientation.w = tf_world_cup.getRotation().w();
      break;
    }
    case State::Dropping:
    {
      m_world_transform.header.frame_id = "world";
      m_world_transform.transform.translation.x = tf_world_cup.getOrigin().x();
      m_world_transform.transform.translation.y = tf_world_cup.getOrigin().y();
      m_world_transform.transform.translation.z = tf_world_cup.getOrigin().z() - CUP_DROP_SPEED;
      m_world_transform.transform.rotation = tf::createQuaternionMsgFromYaw(0);

      m_cup_marker.header.frame_id = "world";
      m_cup_marker.pose.position.x = tf_world_cup.getOrigin().x();
      m_cup_marker.pose.position.y = tf_world_cup.getOrigin().y();
      m_cup_marker.pose.position.z = tf_world_cup.getOrigin().z() - CUP_DROP_SPEED;
      m_cup_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      break;
    }
  }
}