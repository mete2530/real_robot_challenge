#!/usr/bin/env python3

# Original work: official beginner's documentation from Moveit
# 
# Link: https://ros-planning.github.io/moveit_tutorials/doc/motion_planning_pipeline/motion_planning_pipeline_tutorial.html

# Author: Acorn Pooley, Mike Lautman

# Modified by: Xun Tu

# Further modified by Group 10: Zeph Johnson, 

# The main file to execute a saved trajectory for the manipulator


# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input 

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time, random, math
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


from collections import namedtuple
import robot_controller  # Assumed base mobility controller

Obstacle = namedtuple('Obstacle', ['x', 'y', 'radius'])
MapData = namedtuple('MapData', ['obstacles', 'goal'])


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    
    elif isinstance(goal, geometry_msgs.msg.PoseStamped):
        return all_close(goal.pose, actual.pose, tolerance)
    
    elif isinstance(goal, geometry_msgs.msg.Pose):
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = math.dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = abs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
        return d <= tolerance and cos_phi_half >= math.cos(tolerance/2.0)
    
    return True


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
        candidates = [
            (obs.x + ux * obs.radius, obs.y + uy * obs.radius),
            (obs.x - ux * obs.radius, obs.y - uy * obs.radius)
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


class MoveGroupPythonInterfaceSimple(object):
    """A simple MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterfaceSimple, self).__init__()

        ## Setup
        #
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("mani_trajectory_exec", anonymous=True)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states

        # Use the default one
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:

        # Use a simple, default scene
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate two `MoveGroupCommander`_ objects.  The object is an interface
        # to a planning group (group of joints).  

        # In this project there are two groups for the manipulator:
        # One is "arm", the robotic arm
        # The other is "gripper", the gripper at the robot's end-effector
        
        # This interface can be used to plan and execute motions:
        group_name1 = "arm"
        move_group_arm = moveit_commander.MoveGroupCommander(group_name1)

        # Same for the gripper
        group_name2 = "gripper"
        move_group_gripper = moveit_commander.MoveGroupCommander(group_name2)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz: (debugging purpose)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )



        ## Getting Basic Information (Debugging Purpose; de-activated by fault)

        # We can get the name of the reference frame for this robot:
        # planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # # We can also print the name of the end-effector link for this group:
        # eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # # We can get a list of all the groups in the robot:
        # group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # # Sometimes for debugging it is useful to print the entire state of the
        # # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")


        ## Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group_arm = move_group_arm
        self.move_group_gripper = move_group_gripper

        self.display_trajectory_publisher = display_trajectory_publisher
 
        self.group_names = [group_name1, group_name2]

    def go_to_joint_state_arm(self, joint_goal):
        # Move the robotic arm to the desired joint state

        # joint_goal = self.move_group_arm.get_current_joint_values()
        
        # joint_goal[0] = 0
        # joint_goal[1] = 0
        # joint_goal[2] = 0
        # joint_goal[3] = 0
  

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group_arm.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

        # For testing:
        current_joints = self.move_group_arm.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.05)

    def set_gripper_width(self, gripper_width):
        # Move the gripper to the desired width
        #
        # WARNING: when trying to grasp something, but 
        # the width is too small, it might damage the motor 
        # and cause the motor to fail
        # 

        # Similarly, use 'go' to control the gripper
        self.move_group_gripper.go([gripper_width, gripper_width], wait=True)

        # Call 'stop'
        self.move_group_gripper.stop()

        # For testing
        current_gripper_width = self.move_group_gripper.get_current_joint_values()
        return all_close([gripper_width, gripper_width], current_gripper_width, 0.05)

    def pick_butter(self):
        self.set_gripper_width(0.012)
        time.sleep(1)

        print("1. Move down to the lid location")
        trajectory_list = [[-2, 55, -1, -23]]
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(5)
        time.sleep(0.5)

        print("2. grab lid")
        self.set_gripper_width(0.00)

    def drop_butter(self):
        # TODO: Lower, open, and retract
        # self.set_gripper_width(0.012)
        # self.go_to_joint_state_arm([0, 0, 0, 0])
        time.sleep(1)

    def open_butter(self):
        self.set_gripper_width(0.012)
        time.sleep(1)

        print("1. Move down to the lid location")
        trajectory_list = [[-2, 55, -1, -23]]
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(5)
        time.sleep(0.5)

        print("2. grab lid now")
        self.set_gripper_width(0.00)
        
        print("3. move up now")
        trajectory_list = [[0, 0, 0, 0]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(2)

        print("4. turn left now")
        trajectory_list = [[28, 18, -17, -7]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(5)
        time.sleep(2)

        print("5. move down now")
        trajectory_list = [[28, 70, -32, -10]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(5) 
        time.sleep(0.5)

        print("6. release lid now")
        self.set_gripper_width(0.012) 

        print("7. move up")
        trajectory_list = [[28, 18, -17, -7]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(5)
        time.sleep(2)

        print("8. oringinal position")
        trajectory_list = [[0, 0, 0, 0]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(2)

        print("9. move down to butter")
        trajectory_list = [[-2, 55, -1, -23]]
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(5)
        time.sleep(2)

        print("10. grab the butter")
        self.set_gripper_width(-0.006) 

        print("11. move up")
        trajectory_list = [[0, 0, 0, 0]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(5)

        print("11. move right")
        trajectory_list = [[-28, 18, -17, -7]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(5)

        print("12. move down")
        trajectory_list = [[-28, 70, -32, -10]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(2)

        print("13. release butter now")
        self.set_gripper_width(0.012) 

        print("14. move up")
        trajectory_list = [[-28, 18, -17, -7]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(2)

        print("15. move over to the lid")
        trajectory_list = [[28, 18, -17, -7]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(2)

        print("16. move down")
        trajectory_list = [[28, 70, -32, -10]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(5) 
        time.sleep(0.5)

        print("17. grib lid now")
        self.set_gripper_width(0.00) 
        
        print("18. move up")
        trajectory_list = [[28, 18, -17, -7]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(2)

        print("19. move original")
        trajectory_list = [[0, 0, 0, 0]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(2)

        print("20. move down to butter tray")
        trajectory_list = [[-2, 55, -1, -23]]
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(5)
        time.sleep(2)

        print("21. release lid now")
        self.set_gripper_width(0.012) 


        print("22. move original")
        trajectory_list = [[0, 0, 0, 0]] 
        trajectory_list = [[x * pi / 180 for x in wp] for wp in trajectory_list]
        for wp in trajectory_list:
            self.go_to_joint_state_arm(wp)
            time.sleep(2)
        time.sleep(2)


def main():
    rospy.init_node('robot_controller')

    controller = robot_controller.RobotController()
    ## Ensure the robot stops if the node is killed.
    rospy.on_shutdown(controller.stop_robot)

    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the Turtlebot3 Manipulator Control")
        print("Source: MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to start ===================="
        )
        tutlebot3 = MoveGroupPythonInterfaceSimple()
        
        input(
            "============ Press `Enter` to execute pick up butter and generate path..."
        )
        tutlebot3.pick_butter()

        # Navigate to goal using tangent detour path algorithm
        berth = 0.31 # TODO: find appropriate value for robot
        # obstacles = get_obstacles(num_obs=3, berth=berth)
        obstacles = [Obstacle(1.0, 1.0, 0.4)]
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

        input(
            "============ Press `Enter` to execute open butter..."
        )
        tutlebot3.open_butter()



        print("============ Manipulation complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
    rospy.signal_shutdown("Task Completed. Shutting down the node.")


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## END_TUTORIAL
