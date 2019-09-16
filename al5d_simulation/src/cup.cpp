#include "cup.hpp"

Cup::Cup(std::string a_cup_name, Position a_cup_position) : cup_name(a_cup_name), 
  cup_position(a_cup_position), grapped(false)
{
  cup_publisher = n.advertise<visualization_msgs::Marker>(cup_name, 1);

  cup_marker.ns = cup_name;
  cup_marker.id = 0;
  cup_marker.type = visualization_msgs::Marker::CYLINDER;
  cup_marker.action = visualization_msgs::Marker::ADD;

  cup_marker.scale.x = 0.02;
  cup_marker.scale.y = 0.02;
  cup_marker.scale.z = 0.04;

  world_transform.header.frame_id = "world";
  world_transform.child_frame_id = cup_name;

  cup_marker.header.frame_id = "world";
  cup_marker.pose.position.x = cup_position.x_pos;
  cup_marker.pose.position.y = cup_position.y_pos;
  cup_marker.pose.position.z = cup_position.z_pos;
  cup_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  cup_marker.color.a = 1.0;  // Don't forget to set the alpha!
  cup_marker.color.r = 255;
  cup_marker.color.g = 255;
  cup_marker.color.b = 255;

  //   //temporary
  cup_marker.pose.orientation.x = 0;
  cup_marker.pose.orientation.y = 0;
  cup_marker.pose.orientation.z = 0;

  world_transform.header.stamp = ros::Time::now();
  world_transform.transform.translation.x = cup_position.x_pos;
  world_transform.transform.translation.y = cup_position.y_pos;
  world_transform.transform.translation.z = cup_position.z_pos;
  world_transform.transform.rotation = tf::createQuaternionMsgFromYaw(0);

  cup_marker.lifetime = ros::Duration();
  cup_publisher.publish(cup_marker);
  broadcaster.sendTransform(world_transform);
}
Cup::~Cup()
{
}

void Cup::simulate()
{
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    world_transform.header.stamp = ros::Time::now();
    cup_marker.header.stamp = ros::Time::now();

    broadcaster.sendTransform(world_transform);
    publishStatus();
    cup_publisher.publish(cup_marker);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void Cup::publishStatus()
{
  tf::StampedTransform tf_world_cup;
  tf::StampedTransform tf_grippper_right_cup;
  tf::StampedTransform tf_grippper_left_cup;
  try
  {
    listener.waitForTransform("world", cup_name, ros::Time(0), ros::Duration(0.5));
    listener.waitForTransform("gripper_right", cup_name, ros::Time(0), ros::Duration(0.5));
    listener.waitForTransform("gripper_left", cup_name, ros::Time(0), ros::Duration(0.5));
    
    listener.lookupTransform("world", cup_name, ros::Time(0), tf_world_cup);
    listener.lookupTransform("gripper_left", cup_name, ros::Time(0), tf_grippper_left_cup);
    listener.lookupTransform("gripper_right", cup_name, ros::Time(0), tf_grippper_right_cup);

    double diff_right_x = tf_grippper_right_cup.getOrigin().x() * 1000;
    double diff_right_y = tf_grippper_right_cup.getOrigin().y() * 1000;
    double diff_right_z = tf_grippper_right_cup.getOrigin().z() * 1000;

    double diff_left_x = tf_grippper_left_cup.getOrigin().x() * 1000;
    double diff_left_y = tf_grippper_left_cup.getOrigin().y() * 1000;
    double diff_left_z = tf_grippper_left_cup.getOrigin().z() * 1000;

    if (std::abs(diff_right_y) < 23.0 && std::abs(diff_right_y) > 13.0
        && std::abs(diff_left_y) < 23.0 && std::abs(diff_left_y) > 13.0
        && std::abs(diff_right_x) < 18 && std::abs(diff_right_x) > 5.0
        && std::abs(diff_left_x) < 18 && std::abs(diff_left_x) > 5.0
        && std::abs(diff_left_z) < 17.0 && std::abs(diff_left_z) > 2
        && std::abs(diff_right_z) < 17.0 && std::abs(diff_right_z) > 2 )
    {
      cup_state = State::Grabbed;
    }
    else if (cup_position.z_pos < tf_world_cup.getOrigin().z())
    {    
      cup_state = State::Dropped;
    }
    else 
    {
      cup_state = State::Released;
    }

    updateCupState(cup_state, tf_grippper_right_cup, tf_grippper_left_cup, tf_world_cup);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    //ros::Duration(1.0).sleep();
  }
}

void Cup::updateCupState(State cup_state, tf::StampedTransform& tf_grippper_right_cup, tf::StampedTransform& tf_grippper_left_cup, tf::StampedTransform& tf_world_cup )
{
  switch(cup_state)
  {
    case State::Grabbed:
    {
      cup_marker.color.r = 0;
      cup_marker.color.g = 255;
      cup_marker.color.b = 0;
      world_transform.header.frame_id = "gripper_right";
      world_transform.transform.translation.x = tf_grippper_right_cup.getOrigin().x();
      world_transform.transform.translation.y = tf_grippper_right_cup.getOrigin().y();
      world_transform.transform.translation.z = tf_grippper_right_cup.getOrigin().z();
      world_transform.transform.rotation.x = tf_grippper_right_cup.getRotation().x();
      world_transform.transform.rotation.y = tf_grippper_right_cup.getRotation().y();
      world_transform.transform.rotation.z = tf_grippper_right_cup.getRotation().z();
      world_transform.transform.rotation.w = tf_grippper_right_cup.getRotation().w();

      cup_marker.header.frame_id = "gripper_right";
      cup_marker.pose.position.x = tf_grippper_right_cup.getOrigin().x();
      cup_marker.pose.position.y = tf_grippper_right_cup.getOrigin().y();
      cup_marker.pose.position.z = tf_grippper_right_cup.getOrigin().z();
      cup_marker.pose.orientation.x = tf_grippper_right_cup.getRotation().x();
      cup_marker.pose.orientation.y = tf_grippper_right_cup.getRotation().y();
      cup_marker.pose.orientation.z = tf_grippper_right_cup.getRotation().z();
      cup_marker.pose.orientation.w = tf_grippper_right_cup.getRotation().w();
      //std::cout << "Grapped" << std::endl;

      break;
    }
    case State::Released:
    {
      cup_marker.color.r = 255;
      cup_marker.color.g = 255;
      cup_marker.color.b = 255;
      world_transform.header.frame_id = "world";
      world_transform.transform.translation.x = tf_world_cup.getOrigin().x();
      world_transform.transform.translation.y = tf_world_cup.getOrigin().y();
      world_transform.transform.translation.z = tf_world_cup.getOrigin().z();
      world_transform.transform.rotation.x = tf_world_cup.getRotation().x();
      world_transform.transform.rotation.y = tf_world_cup.getRotation().y();
      world_transform.transform.rotation.z = tf_world_cup.getRotation().z();
      world_transform.transform.rotation.w = tf_world_cup.getRotation().w();

      cup_marker.header.frame_id = "world";
      cup_marker.pose.position.x = tf_world_cup.getOrigin().x();
      cup_marker.pose.position.y = tf_world_cup.getOrigin().y();
      cup_marker.pose.position.z = tf_world_cup.getOrigin().z();
      cup_marker.pose.orientation.x = tf_world_cup.getRotation().x();
      cup_marker.pose.orientation.y = tf_world_cup.getRotation().y();
      cup_marker.pose.orientation.z = tf_world_cup.getRotation().z();
      cup_marker.pose.orientation.w = tf_world_cup.getRotation().w();
      break;
    }
    case State::Dropped:
    {
      cup_marker.color.r = 255;
      cup_marker.color.g = 0;
      cup_marker.color.b = 0;
      world_transform.header.frame_id = "world";
      world_transform.transform.translation.x = tf_world_cup.getOrigin().x();
      world_transform.transform.translation.y = tf_world_cup.getOrigin().y();
      world_transform.transform.translation.z = tf_world_cup.getOrigin().z() - 0.01;
      world_transform.transform.rotation = tf::createQuaternionMsgFromYaw(0);

      cup_marker.header.frame_id = "world";
      cup_marker.pose.position.x = tf_world_cup.getOrigin().x();
      cup_marker.pose.position.y = tf_world_cup.getOrigin().y();
      cup_marker.pose.position.z = tf_world_cup.getOrigin().z() - 0.01;
      cup_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      break;
    }
  }
}