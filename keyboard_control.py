#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

rospy.init_node('keyboard_control')
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key

print("Use WASD keys to move")
print("w = forward")
print("s = backward")
print("a = left")
print("d = right")
print("x = stop")
print("q = quit")

twist = Twist()

while not rospy.is_shutdown():

    key = getKey()

    twist.linear.x = 0
    twist.angular.z = 0

    if key == 'w':
        twist.linear.x = 0.5

    elif key == 's':
        twist.linear.x = -0.5

    elif key == 'a':
        twist.angular.z = 1.0

    elif key == 'd':
        twist.angular.z = -1.0

    elif key == 'x':
        twist.linear.x = 0
        twist.angular.z = 0

    elif key == 'q':
        break

    pub.publish(twist)
