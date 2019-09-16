#include "ros/ros.h"
#include <thread>
#include "std_msgs/String.h"
#include "al5d_simulation/servo_position.h"
#include "al5d_simulation/servo_command.h"


using namespace std::chrono_literals;

class VirtualServo
{
public:
  VirtualServo(short a_channel);
  ~VirtualServo();
  void callBack(const al5d_simulation::servo_command& servo);
  ros::Publisher servo_degrees_publisher;
  
private:
  double pwmToDegrees(short incoming_pwm);
  double mapGripper(short incoming_pwm);
  bool doubleEquals(double a, double b);
  void updateGripperPosition(double& current_degrees, double degrees);
  void updateServoPositions(double& current_degrees, double degrees);
  void publishMessage(const al5d_simulation::servo_command& servo);
  short channel;
  short movement_speed;
  short time;
  double current_degrees;
  ros::NodeHandle n;
  ros::Subscriber msg_subscriber;
  al5d_simulation::servo_position servo_position_msg;

};