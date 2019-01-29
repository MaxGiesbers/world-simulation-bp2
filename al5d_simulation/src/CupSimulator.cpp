#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include "Position.hpp"
// #include "Cup.hpp"

int main(int argc, char** argv)
{
  if (argc != 6)
  {
    ROS_ERROR("Not enough arguments are given, %d given", argc);
    return 1;
  }
  geometry_msgs::TransformStamped world_transform;

  // initialize position robotarm
  Position cup_position;
  cup_position.x_pos = atof(argv[1]);
  cup_position.y_pos = atof(argv[2]);
  cup_position.z_pos = atof(argv[3]);

  ros::init(argc, argv, "cupSimulator");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<visualization_msgs::Marker>("cup_1", 1000);
  ros::Rate loop_rate(1);

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "world";
  world_transform.child_frame_id = "my_namespace";


  
  // marker.scale.z = 0.05;
  // marker.color.a = 1.0; // Don't forget to set the alpha!
  // marker.color.r = 1.0;
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;

  marker.pose.position.x = 0.1;
  marker.pose.position.y = 0.1;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.04;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  while (ros::ok())
  {
    chatter_pub.publish(marker);
    ros::spinOnce();
    loop_rate.sleep();

    static tf::TransformListener tfListener;

    tf::StampedTransform tf_world_cup;
    tf::StampedTransform tf_grippper_right_cup;
    tf::StampedTransform tf_grippper_left_cup;

    tfListener.waitForTransform("gripper_right", "my_namespace", ros::Time(0), ros::Duration(0.5));

    // tfListener.lookupTransform("gripper_right", "my_namespace", ros::Time(0), tf_grippper_right_cup);
    // std::cout << tf_grippper_right_cup.getOrigin().x() * 1000 << std::endl;

    std::cout << "print voor beker" << std::endl;
  }
  return 0;
}