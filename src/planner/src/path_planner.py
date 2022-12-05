#!/usr/bin/env python
"""
Path Planner Class for Lab 7
Author: Valmik Prabhu
"""

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

from intera_interface import Limb


class PathPlanner(object):
    """
    Path Planning Functionality for Baxter/Sawyer

    We make this a class rather than a script because it bundles up 
    all the code relating to planning in a nice way thus, we can
    easily use the code in different places. This is a staple of
    good object-oriented programming

    Fields:
    _robot: moveit_commander.RobotCommander; for interfacing with the robot
    _scene: moveit_commander.PlanningSceneInterface; the planning scene stores a representation of the environment
    _group: moveit_commander.MoveGroupCommander; the move group is moveit's primary planning class
    _planning_scene_publisher: ros publisher; publishes to the planning scene


    """
    def __init__(self, group_name): # group of joints name
        """
        Constructor.

        Inputs:
        group_name: the name of the move_group.
            For Baxter, this would be 'left_arm' or 'right_arm'
            For Sawyer, this would be 'right_arm'
        """

        # If the node is shutdown, call this function    
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the robot
        self._robot = moveit_commander.RobotCommander()

        # Initialize the planning scene
        self._scene = moveit_commander.PlanningSceneInterface()

        # This publishes updates to the planning scene
        self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # Instantiate a move group
        self._group = moveit_commander.MoveGroupCommander(group_name)

        # Set the maximum time MoveIt will try to plan before giving up
        self._group.set_planning_time(5)

        # Set the bounds of the workspace (Joint theta of the arm)
        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        # # Set the speed of the commander
        # self._group.set_max_velocity_scaling_factor(1) 		
        # # Set maximum acceleration
        # self._group.set_max_acceleration_scaling_factor(1)

        # Set the maximum link speed
        # Set the maximum Cartesian link speed. Only positive real values are allowed.
        # The unit is  meter per second.
        # limb = Limb("right")
        # for i in limb.joint_names:
        #     self._group.limit_max_cartesian_link_speed(0.01, i)
        
        # Sleep for a bit to ensure that all inititialization has finished
        rospy.sleep(0.5)



    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety

        Currently deletes the object's MoveGroup, so that further commands will do nothing
        """
        self._group = None
        rospy.loginfo("Stopping Path Planner")

    def plan_to_pose(self, waypoints):#, orientation_constraints):
        """
        Generates a plan given an end effector pose subject to orientation constraints

        Inputs:
        target: A geometry_msgs/PoseStamped message containing the end effector pose goal
        orientation_constraints: A list of moveit_msgs/OrientationConstraint messages

        Outputs:
        path: A moveit_msgs/RobotTrajectory path
        """

        # self._group.set_pose_target(target)
        # self._group.set_start_state_to_current_state()

        # constraints = Constraints()
        # constraints.orientation_constraints = orientation_constraints
        # self._group.set_path_constraints(constraints)

        # plan = self._group.plan()

        # compute_cartesian_path(self, waypoints, eef_step(in meters), jump_threshold, avoid_collisions = True)
        # Source: http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#ad7f6d93d73bf43268ba983afb0dc4f23
        plan, _ = self._group.compute_cartesian_path(waypoints, 0.1 ,0)
        velocity_scaling_factor = 0.5
        plan = self._group.retime_trajectory(self._robot.get_current_state(), 
                                plan, 
                                velocity_scaling_factor)
        
        return plan

    def execute_plan(self, plan):
        """
        Uses the robot's built-in controllers to execute a plan

        Inputs:
        plan: a moveit_msgs/RobotTrajectory plan
        """

        return self._group.execute(plan, True)


    # def add_box_obstacle(self, size, name, pose):
    #     """
    #     Adds a rectangular prism obstacle to the planning scene

    #     Inputs:
    #     size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
    #     name: unique name of the obstacle (used for adding and removing)
    #     pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
    #     """    

    #     # Create a CollisionObject, which will be added to the planning scene
    #     co = CollisionObject()
    #     co.operation = CollisionObject.ADD
    #     co.id = name
    #     co.header = pose.header

    #     # Create a box primitive, which will be inside the CollisionObject
    #     box = SolidPrimitive()
    #     box.type = SolidPrimitive.BOX
    #     box.dimensions = size

    #     # Fill the collision object with primitive(s)
    #     co.primitives = [box]
    #     co.primitive_poses = [pose.pose]

    #     # Publish the object
    #     self._planning_scene_publisher.publish(co)

    # def remove_obstacle(self, name):
    #     """
    #     Removes an obstacle from the planning scene

    #     Inputs:
    #     name: unique name of the obstacle
    #     """

    #     co = CollisionObject()
    #     co.operation = CollisionObject.REMOVE
    #     co.id = name

    #     self._planning_scene_publisher.publish(co)