#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, upperarm2forearm=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        // joint_state.header.stamp = ros::Time::now();
        // joint_state.name.resize(3);
        // joint_state.position.resize(3);
        // joint_state.name[0] ="upperarm2forearm";
        // joint_state.position[0] = upperarm2forearm;
        // joint_state.name[1] ="tilt";
        // joint_state.position[1] = tilt;
        // joint_state.name[2] ="periscope";
        // joint_state.position[2] = height;



        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(7);
        joint_state.position.resize(7);
        joint_state.name[0] = "base_link2turret";
        joint_state.position[0] = 0;
        joint_state.name[1] = "turret2upperarm";
        joint_state.position[1] = tilt;
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

        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 1;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0) tinc *= -1;
        // height += hinc;
        // if (height>.2 || height<0) hinc *= -1;
        // swivel += degree;
        // angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}