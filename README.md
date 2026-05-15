# teleopRobot

## Steps to get to the gmapping:

```bash
wget https://lamp.ms.wits.ac.za/robotics/robot_assignment_ws.tar.gz
tar zxvf robot_assignment_ws.tar.gz
cd robot_assignment_ws
catkin_make
source devel/setup.bash
````

---

## Terminal Setup

### Terminal 1:

```bash
./startWorld
```

---

### Terminal 2:

```bash
cd robot_assignment_ws
source devel/setup.bash
roslaunch turtlebot_gazebo gmapping_demo.launch
```

---

### Terminal 3:

```bash
cd ~/robot_assignment_ws/src
catkin_create_pkg my_robot rospy geometry_msgs
cd my_robot
mkdir scripts
cd scripts
nano keyboard_control.py
```

👉 Write the python code, save it, then:

```bash
chmod +x keyboard_control.py
cd ~/robot_assignment_ws
catkin_make
source devel/setup.bash
rosrun my_robot keyboard_control.py
```

---

## If you already have files:

### Terminal 1:

```bash
./startWorld
```

### Terminal 2:

```bash
roslaunch turtlebot_gazebo gmapping_demo.launch
```

### Terminal 3:

```bash
rosrun my_robot keyboard_control.py
```

---

## Controls

You keep the mouse on Terminal 3 while moving your robot around to build the map.

When moving the robot, there is the robot’s name and a turtle marker that looks like a key. The head of the key shows the front of the robot.

The robot only has “front vision”, so you must rotate it to scan the environment properly.

### Keyboard controls:

* **W** → Move forward
* **S** → Move backward
* **A** → Turn left
* **D** → Turn right

👉 Move slowly — do not rush, or the map will become inaccurate.

---

## Saving your map

To save the generated map:

```bash
rosrun map_server map_saver -f surveillance_map
```

This creates:

* `surveillance_map.pgm`
* `surveillance_map.yaml`

---

## Notes

* Always run `source devel/setup.bash` in every new terminal
* Make sure all terminals are in the correct workspace
* Slow movement improves mapping quality

```

---

