#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

rospy.init_node('navigate_to_goal')

cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

goal_x = float(input("Enter goal x: "))
goal_y = float(input("Enter goal y: "))

rate = rospy.Rate(10)

while not rospy.is_shutdown():

    state = get_state('turtlebot', '')

    x = state.pose.position.x
    y = state.pose.position.y

    orientation = state.pose.orientation

    siny_cosp = 2 * (orientation.w * orientation.z +
                     orientation.x * orientation.y)

    cosy_cosp = 1 - 2 * (orientation.y ** 2 +
                         orientation.z ** 2)

    yaw = math.atan2(siny_cosp, cosy_cosp)

    dx = goal_x - x
    dy = goal_y - y

    distance = math.sqrt(dx**2 + dy**2)

    target_angle = math.atan2(dy, dx)

    angle_error = target_angle - yaw

    twist = Twist()

    if distance > 0.1:

        twist.linear.x = 0.2
        twist.angular.z = 1.0 * angle_error

    else:

        twist.linear.x = 0
        twist.angular.z = 0

        print("Goal reached!")
        cmd_pub.publish(twist)
        break

    cmd_pub.publish(twist)

    rate.sleep()
