# world-simulation-bp2

Start simulatie commando:
roslaunch al5d_simulation display.launch 

Bug geen robot commando: 

export LC_NUMERIC="en_US.UTF-8"



path to text file: 
Run publisher

rosrun al5d_simulation Publisher src/world-simulation-bp2/al5d-simulation/config/commands.txt



info over de servos
// servoList.push_back(Lowlevel::Servo("Base", -90, 90, 500, 2500));
    // servoList.push_back(Lowlevel::Servo("Shoulder", -30, 90, 1100, 2500));
    // servoList.push_back(Lowlevel::Servo("Elbow", 0, 135, 650, 2500));
    // servoList.push_back(Lowlevel::Servo("Wrist", -90, 90, 500, 2500));
    // servoList.push_back(Lowlevel::Servo("WristRotate", -90, 90, 500, 2500));
    // servoList.push_back(Lowlevel::Servo("Gripper", -90, 90, 500, 2500));

