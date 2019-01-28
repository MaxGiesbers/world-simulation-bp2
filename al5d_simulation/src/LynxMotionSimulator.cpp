#include "LynxMotionSimulator.hpp"

LynxMotionSimulator::LynxMotionSimulator():degree(M_PI / 180)
{
  servo_subscriber = n.subscribe("servo_degrees", 1000, &LynxMotionSimulator::publishCommands, this);
  joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  world_transform.header.frame_id = "world";
  world_transform.child_frame_id = "base_link";
}

LynxMotionSimulator::~LynxMotionSimulator()
{
}

void LynxMotionSimulator::publishCommands(const al5d_simulation::servo_command& servo_degrees)
{
  
  std::cout << "received command:" << servo_degrees.degrees << std::endl;

  short degrees = servo_degrees.degrees;




//   name,base,shoulder,elbow,wrist,wristrotate,gripper,velocity,
// PARK,0,47,98,-45,0,0,2000,
// STRAIGHT,0,0,0,0,0,0,2000,
// READY,0,40,80,5,0,0,2000

  double testValue = degrees * M_PI / 180.0;
  

  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(7);
  joint_state.position.resize(7);
  joint_state.name[0] = "base_link2turret";
  joint_state.position[0] = 0;
  joint_state.name[1] = "turret2upperarm";
  joint_state.position[1] = testValue;
  joint_state.name[2] = "upperarm2forearm";
  joint_state.position[2] = 0;
  joint_state.name[3] = "forearm2wrist";
  joint_state.position[3] = 0;
  joint_state.name[4] = "wrist2hand";
  joint_state.position[4] = 0;
  joint_state.name[5] = "gripper_left2hand";
  joint_state.position[5] = 0;
  joint_state.name[6] = "gripper_right2hand";
  joint_state.position[6] = 0;


  world_transform.header.stamp = ros::Time::now();
  world_transform.transform.translation.x = 0.0;
  world_transform.transform.translation.y = 0.0;
  world_transform.transform.translation.z = 0.0;


  world_transform.transform.rotation = tf::createQuaternionMsgFromYaw(180 * degree);

  // send the joint state and transform
  joint_publisher.publish(joint_state);
  broadcaster.sendTransform(world_transform);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lynx_motion_simulator");
  LynxMotionSimulator simulator;
  std::cout << "print" << std::endl;

  ros::spin();
  return 0;
}
