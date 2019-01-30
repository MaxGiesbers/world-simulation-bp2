#include "Cup.hpp"

Cup::Cup(std::string a_cup_name, Position cup_position) : cup_name(a_cup_name)
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
  cup_marker.color.r = 0.0;
  cup_marker.color.g = 1.0;
  cup_marker.color.b = 0.0;

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

void Cup::publishStatus()
{
//   world_transform.header.stamp = ros::Time::now();
  

  //   tf::StampedTransform tf_world_cup;

  //   try
  //   {
  //     listener.waitForTransform("world", cup_name, ros::Time(0), ros::Duration(0.5));
  //     listener.waitForTransform("gripper_right", cup_name, ros::Time(0), ros::Duration(0.5));
  //     listener.lookupTransform("world", cup_name, ros::Time(0), tf_world_cup);

  //     // if (listener.canTransform("gripper_right", cup_name, ros::Time(0)))
  //     // {
  //     listener.lookupTransform("gripper_right", cup_name, ros::Time(0), transform);
  //     //   std::cout << transform.getOrigin().x() * 1000 << std::endl;
  //     //   std::cout << "komt in in lookup" << std::endl;
  //   }
  //   catch (tf::TransformException ex)
  //   {
  //     ROS_ERROR("%s", ex.what());
  //     ros::Duration(1.0).sleep();
  //   }

  tf::StampedTransform tf_world_cup;
  tf::StampedTransform tf_grippper_right_cup;
  tf::StampedTransform tf_grippper_left_cup;
  ros::Time t = ros::Time(0);

  try
  {
    // listener.waitForTransform("world", cup_name, ros::Time(0), ros::Duration(0.5));
    // listener.waitForTransform("gripper_right", cup_name, ros::Time(0), ros::Duration(0.5));
    // listener.waitForTransform("gripper_left", cup_name, ros::Time(0), ros::Duration(0.5));
    listener.lookupTransform("world", cup_name, t, tf_world_cup);

    // if (listener.canTransform("gripper_right", cup_name, ros::Time(0)))
    // {
    listener.lookupTransform("gripper_right", cup_name, t, tf_grippper_right_cup);
    std::cout << "can transform?" << std::endl;
    // }
    // if (listener.canTransform("gripper_left", cup_name, ros::Time(0)))
    // {
    //   listener.lookupTransform("gripper_left", cup_name, ros::Time(0), tf_grippper_left_cup);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  broadcaster.sendTransform(world_transform);
  cup_publisher.publish(cup_marker);
}