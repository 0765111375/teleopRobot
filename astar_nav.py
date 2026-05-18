#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import heapq
import sys

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState


# =========================================
# GLOBALS
# =========================================
map_data = None
map_info = None


# =========================================
# PARAMETERS
# =========================================
MAX_SPEED = 0.35
MIN_SPEED = 0.10
TURN_GAIN = 2.2
GOAL_TOLERANCE = 0.25
LOOKAHEAD_DISTANCE = 0.6
INFLATION_RADIUS = 4


# =========================================
# MAP CALLBACK
# =========================================
def map_callback(msg):
    global map_data, map_info

    raw = np.array(msg.data).reshape(
        (msg.info.height, msg.info.width)
    )

    cleaned = np.copy(raw)

    # unknown = obstacle
    cleaned[cleaned == -1] = 100

    map_data = cleaned
    map_info = msg.info


# =========================================
# INFLATE OBSTACLES
# =========================================
def inflate(grid, radius=4):

    h, w = grid.shape
    inflated = np.copy(grid)

    obstacles = np.argwhere(grid >= 50)

    for oy, ox in obstacles:

        y1 = max(0, oy - radius)
        y2 = min(h, oy + radius + 1)

        x1 = max(0, ox - radius)
        x2 = min(w, ox + radius + 1)

        inflated[y1:y2, x1:x2] = 100

    return inflated


# =========================================
# HEURISTIC
# =========================================
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# =========================================
# A* PLANNER
# =========================================
def astar(start, goal, grid):

    h, w = grid.shape

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}

    g_score = {start: 0}

    visited = set()

    while open_set:

        _, current = heapq.heappop(open_set)

        if current in visited:
            continue

        visited.add(current)

        if current == goal:
            break

        x, y = current

        neighbors = [

            (x + 1, y),
            (x - 1, y),

            (x, y + 1),
            (x, y - 1),

            (x + 1, y + 1),
            (x - 1, y - 1),

            (x + 1, y - 1),
            (x - 1, y + 1)
        ]

        for nx, ny in neighbors:

            if 0 <= nx < w and 0 <= ny < h:

                # obstacle
                if grid[ny][nx] != 0:
                    continue

                move_cost = 1.4 if nx != x and ny != y else 1.0

                new_cost = g_score[current] + move_cost

                if (nx, ny) not in g_score or \
                        new_cost < g_score[(nx, ny)]:

                    g_score[(nx, ny)] = new_cost

                    priority = new_cost + heuristic(
                        (nx, ny),
                        goal
                    )

                    heapq.heappush(
                        open_set,
                        (priority, (nx, ny))
                    )

                    came_from[(nx, ny)] = current

    # reconstruct path
    path = []

    if current != goal:
        return []

    while current in came_from:

        path.append(current)

        current = came_from[current]

    path.reverse()

    return path


# =========================================
# REDUCE PATH
# =========================================
def reduce_path(path, step=10):

    if len(path) < step:
        return path

    reduced = path[::step]

    if reduced[-1] != path[-1]:
        reduced.append(path[-1])

    return reduced


# =========================================
# WORLD -> GRID
# =========================================
def world_to_grid(x, y):

    gx = int(
        (x - map_info.origin.position.x)
        / map_info.resolution
    )

    gy = int(
        (y - map_info.origin.position.y)
        / map_info.resolution
    )

    return gx, gy


# =========================================
# GRID -> WORLD
# =========================================
def grid_to_world(gx, gy):

    x = (
        gx * map_info.resolution
        + map_info.origin.position.x
    )

    y = (
        gy * map_info.resolution
        + map_info.origin.position.y
    )

    return x, y


# =========================================
# GET ROBOT POSE
# =========================================
def get_robot_pose(get_state):

    state = get_state('mobile_base', '')

    x = state.pose.position.x
    y = state.pose.position.y

    q = state.pose.orientation

    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )

    return x, y, yaw


# =========================================
# NORMALIZE ANGLE
# =========================================
def normalize_angle(a):
    return math.atan2(
        math.sin(a),
        math.cos(a)
    )


# =========================================
# SAFETY CHECK
# =========================================
def is_safe_world(x, y, grid):

    gx, gy = world_to_grid(x, y)

    h, w = grid.shape

    if gx < 0 or gy < 0 or gx >= w or gy >= h:
        return False

    return grid[gy][gx] == 0


# =========================================
# MAIN
# =========================================
def main():

    global map_data

    rospy.init_node("astar_navigation")

    rospy.Subscriber(
        "/map",
        OccupancyGrid,
        map_callback
    )

    cmd_pub = rospy.Publisher(
        "/cmd_vel_mux/input/teleop",
        Twist,
        queue_size=10
    )

    rospy.wait_for_service("/gazebo/get_model_state")

    get_state = rospy.ServiceProxy(
        "/gazebo/get_model_state",
        GetModelState
    )

    rate = rospy.Rate(20)

    print("Waiting for map...")

    while map_data is None:
        rate.sleep()

    print("Map received!")

    # safer map
    safe_map = inflate(
        map_data,
        radius=INFLATION_RADIUS
    )

    # =====================================
    # USER INPUT
    # =====================================
    print("Enter goal X:")
    sys.stdout.flush()
    goal_x = float(sys.stdin.readline())

    print("Enter goal Y:")
    sys.stdout.flush()
    goal_y = float(sys.stdin.readline())

    # =====================================
    # START / GOAL
    # =====================================
    start_x, start_y, _ = get_robot_pose(get_state)

    start = world_to_grid(start_x, start_y)
    goal = world_to_grid(goal_x, goal_y)

    # =====================================
    # GOAL VALIDATION
    # =====================================
    if not is_safe_world(goal_x, goal_y, safe_map):

        print("Goal is inside obstacle or unsafe area")

        return

    print("Planning path...")

    path = astar(start, goal, safe_map)

    if not path:

        print("No path found!")

        return

    # reduce waypoints
    path = reduce_path(path, step=10)

    waypoints = []

    for gx, gy in path:
        wx, wy = grid_to_world(gx, gy)
        waypoints.append((wx, wy))

    print("Path found!")
    print("Following path...")

    # =====================================
    # FOLLOW WAYPOINTS
    # =====================================
    for wx, wy in waypoints:

        while not rospy.is_shutdown():

            x, y, yaw = get_robot_pose(get_state)

            dx = wx - x
            dy = wy - y

            distance = math.sqrt(dx*dx + dy*dy)

            # waypoint reached
            if distance < GOAL_TOLERANCE:
                break

            target_angle = math.atan2(dy, dx)

            error = normalize_angle(
                target_angle - yaw
            )

            # =================================
            # LOOKAHEAD COLLISION CHECK
            # =================================
            future_x = x + LOOKAHEAD_DISTANCE * math.cos(yaw)
            future_y = y + LOOKAHEAD_DISTANCE * math.sin(yaw)

            cmd = Twist()

            if not is_safe_world(
                    future_x,
                    future_y,
                    safe_map):

                # curve around obstacle
                cmd.linear.x = 0.08
                cmd.angular.z = 1.2

                cmd_pub.publish(cmd)

                rate.sleep()

                continue

            # =================================
            # SMOOTH FAST CONTROLLER
            # =================================
            speed = MIN_SPEED + \
                MAX_SPEED * max(0, 1 - abs(error))

            speed = min(MAX_SPEED, speed)

            cmd.linear.x = speed

            cmd.angular.z = max(
                -1.5,
                min(1.5, TURN_GAIN * error)
            )

            cmd_pub.publish(cmd)

            rate.sleep()

    print("Goal reached!")

    cmd_pub.publish(Twist())


# =========================================
# START
# =========================================
if __name__ == "__main__":
    main()
