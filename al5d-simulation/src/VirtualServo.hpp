class VirtualServo
{
private:
  short channel;
  short pwm;
  short movement_speed;
  short time;

public:
  VirtualServo();
  ~VirtualServo();

  void setChannel(short a_channel);
  void setPwm(short a_pwm);
  void setMovementSpeed(short a_movement_speed);
  void setTime(short a_time);
};