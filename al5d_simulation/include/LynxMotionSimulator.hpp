#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "al5d_simulation/servo_command.h"
#include "RobotArmPosition.hpp"

class LynxMotionSimulator {

    public:
    LynxMotionSimulator();
    ~LynxMotionSimulator();
    void publishCommands( const al5d_simulation::servo_command& servo_degrees);
    ros::Subscriber servo_subscriber;
    ros::Publisher joint_publisher;

    private:

    geometry_msgs::TransformStamped world_transform;
    tf::TransformBroadcaster broadcaster;
    sensor_msgs::JointState joint_state;
    const double degree;
    RobotArmPosition robot_arm_position;

    ros::NodeHandle n;


};