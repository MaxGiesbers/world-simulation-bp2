#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "al5d_simulation/servo_command.h"



class LynxMotionSimulator {

    public:
    LynxMotionSimulator();
    ~LynxMotionSimulator();
    void publishCommands( const al5d_simulation::servo_command& servo_degrees);
    ros::Subscriber servo_subscriber;
    ros::Publisher joint_publisher;

    private:
    
    ros::NodeHandle n;


};