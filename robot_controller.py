#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import time, random, math
import numpy as np

from geometry_msgs.msg import Twist

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from collections import namedtuple


Obstacle = namedtuple('Obstacle', ['x', 'y', 'radius'])
MapData = namedtuple('MapData', ['obstacles', 'goal'])

class RobotController():
    def __init__(self,
                 kp_linear=1.0,
                 kp_angular=1.0,
                 dist_threshold=0.05,
                 angle_threshold=3*np.pi/180,
                 max_linear_speed=0.1,
                 max_angular_speed=0.5):
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular
        self.dist_threshold = dist_threshold
        self.angle_threshold = angle_threshold
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

        ## Transform Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        ## Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20)

    @staticmethod
    def normalize_angle(angle):
        '''Helper function to Normalize the angle between -pi to pi.'''
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def move_to_waypoints(self, waypoints : list):
        '''Move the robot to the given waypoints.
        waypoints : List of tuples (x, y, theta)'''
        for pose in waypoints:
            self.move_to_pose(pose[0], pose[1], pose[2])

    def move_to_pose(self, x, y, theta):

        vel_msg = Twist()

        while not rospy.is_shutdown():
            ## Get the current pose.
            try:
                current_pose = self.tf_buffer.lookup_transform('map', 'robot_base', rospy.Time(0), rospy.Duration(5))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to get the current pose.")
                continue

            current_trans = current_pose.transform.translation
            current_rot = current_pose.transform.rotation

            _, _, yaw = tf.transformations.euler_from_quaternion([current_rot.x, current_rot.y, current_rot.z, current_rot.w])

            x_current = current_trans.x
            y_current = current_trans.y
            theta_current = self.normalize_angle(yaw)

            ## Compute the position errors
            dx = x - x_current
            dy = y - y_current
            distance_to_goal = np.sqrt(dx**2 + dy**2)

            ## Compute the angle error
            angle_to_goal = np.arctan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - theta_current)

            rospy.loginfo(f"Current: {x_current:.3f}, {y_current:.3f}, {theta_current:.3f} | Goal: {x:.3f}, {y:.3f}, {theta:.3f} | Dist: {distance_to_goal:.3f}, Angle: {angle_error:.3f}")

            if distance_to_goal > self.dist_threshold:
                ## Scale the linear speed based on the angle error.
                linear_speed_scaling_factor = min(1, np.exp(-5*abs(angle_error)))
                vel_msg.linear.x = np.clip(self.kp_linear * linear_speed_scaling_factor, 0, self.max_linear_speed)
                vel_msg.angular.z = np.clip(self.kp_angular * angle_error, -self.max_angular_speed, self.max_angular_speed)

            else:
                ## Close to the goal.
                vel_msg.linear.x = 0.0

                ## Calculate the angle error using arctan2 to avoid discontinuity at -pi and pi.
                angle_error = self.normalize_angle(theta - theta_current)

                ## Align with the goal orientation.
                if abs(angle_error) > self.angle_threshold:
                    vel_msg.angular.z = np.clip(self.kp_angular * angle_error, -self.max_angular_speed, self.max_angular_speed)
                else:
                    self.stop_robot()
                    rospy.loginfo("Goal Reached. Stopping the robot.")
                    break
            
            self.cmd_vel_pub.publish(vel_msg)
            self.rate.sleep()

    def stop_robot(self):
        '''Stop the robot.'''
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)


def get_random(range = 1.8, min = 0.0):
    return random.uniform(min, min+math.abs(range))


def get_obstacles(num_obs=3, x_range=1.8, y_range=1.8, berth=0.4):
    # generates obstacles of random position and radius.
    # includes the robot's radius in obstacle radius to simplify calculations
    # TODO: tweak map data
    obstacles = []
    for _ in range(num_obs):
        x = get_random(range = x_range)
        y = get_random(range = y_range)
        r = get_random(range = 0.2, min = berth)
        obstacles.append(Obstacle(x, y, r))
    # goal = (get_random(range = x_range), get_random(range = y_range), 0.0)
    # goal = (1.8, 1.8, 0.0)

    return obstacles


def line_circle_intersect(a, b, obs):
    # distance from circle center to line segment ab
    (x1, y1) = (a[0], a[1])
    (x2, y2) = (b[0], b[1])
    dx = x2-x1
    dy = y2-y1
    t = ((obs.x-x1)*dx + (obs.y-y1)*dy) / (dx*dx + dy*dy)
    t = max(0, min(1, t))
    px = x1 + t*dx
    py = y1 + t*dy
    return math.hypot(px-obs.x, py-obs.y) < obs.radius


def compute_heading(from_pt, to_pt):
    dx = to_pt[0] - from_pt[0]
    dy = to_pt[1] - from_pt[1]
    return math.atan2(dy, dx)


def generate_tangent_detour_path(start, goal, obstacles, depth=5):
    """
    Returns an ordered list of waypoints (x,y,theta) of a valid path (excluding start).
    Returns None if no valid path was found (due to out of recursion budget or no path possible),
    Recursively computes a collision‐free path from start to goal around circular obstacles
    by adding an intermediary tangent point to avoid the first obstacle collision.
    """

    # Base case: Failure if we've recursed too deeply
    if depth < 0:
        return None

    # Search through obstacles and check if they intersect
    for obs in obstacles:
        if not line_circle_intersect(start, goal, obs):
            continue

        # Compute unit perpendicular to the start to goal vector
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        L = math.hypot(dx, dy)
        ux, uy = -dy / L, dx / L  # perpendicular unit vector

        # Tangent candidate for going around on left and right respectively
        clearance = obs.radius + 0.1
        candidates = [
            (obs.x + ux * clearance, obs.y + uy * clearance),
            (obs.x - ux * clearance, obs.y - uy * clearance)
        ]

        # Try both candidates
        for cx, cy in candidates:
            theta = compute_heading((cx, cy), goal)
            wp = (cx, cy, theta)
            # Recursively find path for each leg
            first_leg = generate_tangent_detour_path(start, wp, obstacles, depth-1)
            if first_leg is None:
                continue
            second_leg = generate_tangent_detour_path(wp, goal, obstacles, depth-1)
            if second_leg is None:
                continue
            # Both legs succeeded, return the found path
            return first_leg + second_leg
        
        # Neither candidate worked, return failure
        return None
    
    # No collisions detected, return base case path
    return [goal]


# def generate_bug_path(start, goal, obstacles):
#     path = [start]
#     for obs in obstacles:
#         if line_circle_intersect(start, goal, obs):
#             # simple detour: go to tangent point at obstacle boundary
#             theta = compute_heading((obs.x, obs.y), goal)
#             tx = obs.x + obs.radius * math.cos(theta)
#             ty = obs.y + obs.radius * math.sin(theta)
#             path.append((tx, ty, compute_heading((tx, ty), goal)))
#     path.append(goal)
#     return path


def main():
    rospy.init_node('robot_controller')

    controller = RobotController()

    ## Ensure the robot stops if the node is killed.
    rospy.on_shutdown(controller.stop_robot)

    # Navigate to goal using tangent detour path algorithm
    berth = 0.31 # TODO: find appropriate value for robot
    # obstacles = get_obstacles(num_obs=3, berth=berth)
    obstacles = [Obstacle(1.5, 0.1, 0.4)]
    start = (0.1, 0.1, 0.0)
    goal = (1.5, 1.5, 0.0)
    path = generate_tangent_detour_path(start, goal, obstacles)

    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print("Generated obstacles:")
    for i, obstacle in enumerate(obstacles):
        x, y, r = obstacle
        print(f"  Obstacle {i}:  x={x:.3f}, y={y:.3f}, r={r:.3f}")


    print("Generated tangent detour path:")
    for i, waypoint in enumerate(path):
        x, y, theta = waypoint
        print(f"  Waypoint {i}:  x={x:.3f}, y={y:.3f}, θ={theta:.3f}")

    input(
        "============ Press `Enter` to execute tangent detour path to goal..."
    )
    controller.move_to_waypoints(path)
    # for waypoint in path:
    #     x, y, theta = waypoint
    #     robot_controller.move_to_pose(x, y, theta)
    #     time.sleep(1)

    rospy.signal_shutdown("Task Completed. Shutting down the node.")

if __name__ == '__main__':
    main()