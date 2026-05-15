# teleopRobot
Steps to get to the gmapping:
  wget https://lamp.ms.wits.ac.za/robotics/robot_assignment_ws.tar.gz
  tar zxvf robot_assignment_ws.tar.gz
  cd robot_assignment_ws
  catkin_make
  source devel/setup.bash
  
  Terminal 1:
    ./startWorld
  Terminal 2
    cd robot_assignment_ws
    source devel/setup.bash
    roslaunch turtlebot_gazebo gmapping_demo.launch
  Terminal 3:
    cd ~/robot_assignment_ws/src
    catkin_create_pkg my_robot rospy geometry_msgs
    cd my_robot
    mkdir scripts
    cd scripts
    nano keyboard_control.py -> write the python code and save it 
    chmod +x keyboard_control.py
    cd ~/robot_assignment_ws
    catkin_make
    source devel/setup.bash
    rosrun my_robot keyboard_control.py
    
