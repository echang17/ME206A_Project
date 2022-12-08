#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback
import tf2_ros

# from moveit_msgs.msg import OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped, Pose
# from geometry_msgs.msg import Pose
from vision.msg import VisualData
from vision.msg import SawyerCog

from path_planner import PathPlanner

try:
    from controller import Controller
except ImportError:
    pass
    
def execute_plan(Visual_Data):
    """
    Main Script
    """
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")


    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    my_controller = Controller(Kp, Kd, Ki, Kw, Limb("right"))

    # COG Stuff:
    width = Visual_Data.obj_width
    length = Visual_Data.obj_length

    el1 = Visual_Data.m1
    el2 = Visual_Data.m2
    el3 = Visual_Data.m3
    el4 = Visual_Data.m4
    el5 = Visual_Data.human_ar

    m1_pos = np.array([el1.pose.position.x, el1.pose.position.y, el1.pose.position.z])
    m2_pos = np.array([el2.pose.position.x, el2.pose.position.y, el2.pose.position.z])
    m3_pos = np.array([el3.pose.position.x, el3.pose.position.y, el3.pose.position.z])
    m4_pos = np.array([el4.pose.position.x, el4.pose.position.y, el4.pose.position.z])
    hand_pos = np.array([el5.pose.position.x, el5.pose.position.y, el5.pose.position.z])

    dist = m2_pos[0] - hand_pos[0]
    sawyer_x_cog = m4_pos[0] + dist
    sawyer_x_init = (m4_pos[0] + m3_pos[0]) / 2
    sawyer_y = m2_pos[1] + Visual_Data.obj_length
    sawyer_z = m2_pos[2]

    gripper_offset = 0.1

    sawyer_cog = Pose()
    sawyer_cog.position.x = sawyer_x_cog
    sawyer_cog.position.y = sawyer_y - gripper_offset
    sawyer_cog.position.z = sawyer_z

    width = Visual_Data.obj_width
    length = Visual_Data.obj_length
    underside_offset = 0.1

    sawyer_init = Pose()
    sawyer_init.position.x = sawyer_x_init
    sawyer_init.position.y = sawyer_y - gripper_offset
    sawyer_init.position.z = sawyer_z
    
    sawyer_init.orientation.x = Visual_Data.m1.pose.orientation.x
    sawyer_init.orientation.y = Visual_Data.m1.pose.orientation.y#-1.0 * Visual_Data.human_ar.pose.orientation.y
    sawyer_init.orientation.z = Visual_Data.m1.pose.orientation.z
    sawyer_init.orientation.w = Visual_Data.m1.pose.orientation.w


    # # 
    # # Add the obstacle to the planning scene here
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # WHat orientation do we want for the gripper?
    # Use AR Tag info
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.x = 0.0;
    orien_const.orientation.y = 0.7;
    orien_const.orientation.z = -0.7;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    # Initial Position:
    initial_pose = sawyer_init #.sawyer_init

    # Publish Obstacles:
    # load object
    load = PoseStamped()
    load.pose.position = sawyer_cog.position
    load.pose.orientation = sawyer_init.orientation
    
    # IS THIS THE RIGHT ORDER????
    size_array = np.array([width, length, 0.25])
    name = 'load'
    


    # Create a CollisionObject, which will be added to the planning scene
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = 'load_object_header'

    # Create a box primitive, which will be inside the CollisionObject
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = size_array

    # Fill the collision object with primitive(s)
    co.primitives = [box]
    co.primitive_poses = [load.pose]
    planner.add_box_obstacle(size_array, 'load_object_header', load.pose)


    # ############################
    # # Line-of-sight obstacles: #
    # ############################

    # # Camera Pose:
    # transform = tfBuffer.lookup_transform('reference/base', 'head_camera', rospy.Time()).transform  
    # translation = transform.translation
    # translation_w = [translation.x, translation.y, translation.z]
    # rot = transform.rotation
    # cam_loc = PoseStamped()
    # cam_loc.pose.orientation = rot
    # cam_loc.pose.position = translation
    

    

    # # Obstacle vectors relative to head_camera frame:
    # tag1_diff = m1_pos - translation_w
    # tag2_diff = m2_pos - translation_w
    # tag3_diff = m3_pos - translation_w
    # tag4_diff = m4_pos - translation_w

    # # Distances of ar_tags from camera
    # m1_dist = np.sqrt(np.sum(tag1_diff))
    # m2_dist = np.sqrt(np.sum(tag2_diff))
    # m3_dist = np.sqrt(np.sum(tag3_diff))
    # m4_dist = np.sqrt(np.sum(tag4_diff)) 

    # # cog of obstacle rectangles relative to base frame:
    # cog_el1 = m1_dist/2*(m1_pos + translation_w)
    # cog_el2 = m2_dist/2*(m2_pos + translation_w)
    # cog_el3 = m3_dist/2*(m3_pos + translation_w)
    # cog_el4 = m4_dist/2*(m4_pos + translation_w)

    # # Vision obstacle sizes:
    # # IS THIS THE RIGHT ORDER????
    # size_array_v1 = np.array([m1_dist, 0.01, 0.01])
    # size_array_v2 = np.array([m2_dist, 0.01, 0.01])
    # size_array_v3 = np.array([m3_dist, 0.01, 0.01])
    # size_array_v4 = np.array([m4_dist, 0.01, 0.01])
    
    # # Needed: position/orientation of cogs, and size of obstacle

    # # RAY 1
    # co_vi_1 = CollisionObject()
    # co_vi_1.operation = CollisionObject.ADD
    # co_vi_1.id = 'vision_obstacle_1'
    # co_vi_1.header = 'vision_obstacle_1'

    # # Create a box primitive, which will be inside the CollisionObject
    # vi1 = SolidPrimitive()
    # vi1.type = SolidPrimitive.BOX
    # vi1.dimensions = size_array_v1

    # # Fill the collision object with primitive(s)
    # co_vi_1.primitives = [vi1]
    # vi1p = PoseStamped()
    # m1_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_1', rospy.Time()).transform.rotation
    
    
    # vi1p.pose.position.x = cog_el1[0]
    # vi1p.pose.position.y = cog_el1[1]
    # vi1p.pose.position.z = cog_el1[2]
    # vi1p.pose.orientation = m1_vec_quat
    # planner.add_box_obstacle(size_array_v1, 'vision_obstacle_1', vi1p.pose)

    # # RAY 2
    # co_vi_2 = CollisionObject()
    # co_vi_2.operation = CollisionObject.ADD
    # co_vi_2.id = 'vision_obstacle_2'
    # co_vi_2.header = 'vision_obstacle_2'

    # # Create a box primitive, which will be inside the CollisionObject
    # vi2 = SolidPrimitive()
    # vi2.type = SolidPrimitive.BOX
    # vi2.dimensions = size_array_v2

    # # Fill the collision object with primitive(s)
    # co_vi_2.primitives = [vi2]
    # vi2p = PoseStamped()
    # m2_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_2', rospy.Time()).transform.rotation
    # vi2p.pose.position.x = cog_el2[0]
    # vi2p.pose.position.y = cog_el2[1]
    # vi2p.pose.position.z = cog_el2[2]
    # vi2p.pose.orientation = m2_vec_quat
    # planner.add_box_obstacle(size_array_v2, 'vision_obstacle_2', vi2p.pose)

    # # RAY 3 
    # co_vi_3 = CollisionObject()
    # co_vi_3.operation = CollisionObject.ADD
    # co_vi_3.id = 'vision_obstacle_3'
    # co_vi_3.header = 'vision_obstacle_3'

    # # Create a box primitive, which will be inside the CollisionObject
    # vi3 = SolidPrimitive()
    # vi3.type = SolidPrimitive.BOX
    # vi3.dimensions = size_array_v3

    # # Fill the collision object with primitive(s)
    # co_vi_3.primitives = [vi3]
    # vi3p = PoseStamped()
    # m3_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_3', rospy.Time()).transform.rotation
    # vi3p.pose.position.x = cog_el3[0]
    # vi3p.pose.position.y = cog_el3[1]
    # vi3p.pose.position.z = cog_el3[2]
    # vi3p.pose.orientation = m3_vec_quat
    # planner.add_box_obstacle(size_array_v3, 'vision_obstacle_3', vi3p.pose)

    # # RAY 4
    # co_vi_4 = CollisionObject()
    # co_vi_4.operation = CollisionObject.ADD
    # co_vi_4.id = 'vision_obstacle_4'
    # co_vi_4.header = 'vision_obstacle_4'

    # # Create a box primitive, which will be inside the CollisionObject
    # vi4 = SolidPrimitive()
    # vi4.type = SolidPrimitive.BOX
    # vi4.dimensions = size_array_v4

    # # Fill the collision object with primitive(s)
    # co_vi_4.primitives = [vi4]
    # vi4p = PoseStamped()
    # m4_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_4', rospy.Time()).transform.rotation
    # vi4p.pose.position.x = cog_el4[0]
    # vi4p.pose.position.y = cog_el4[1]
    # vi4p.pose.position.z = cog_el4[2]
    # vi4p.pose.orientation = m4_vec_quat
    # planner.add_box_obstacle(size_array_v4, 'vision_obstacle_4', vi4p.pose)

    while not rospy.is_shutdown():
        try:
            # x, y, z = 0.8, 0.05, 0.07
            # goal_1 = initial_pose
            # x,y,z = initial_pose[0],initial_pose[1],initial_pose[2]
            # goal_1.header.frame_id = "base"

            # #x, y, and z position
            goal_1 = PoseStamped()
            goal_1.pose = initial_pose#.position.x = x
            # goal_1.pose.position.y = y
            # goal_1.pose.position.z = z

            # #Orientation as a quaternion
            goal_1.pose.orientation.x = 0.0
            goal_1.pose.orientation.y = 0.7
            goal_1.pose.orientation.z = -0.7
            # goal_1.pose.orientation.x = 0.0
            # goal_1.pose.orientation.y = -1.0
            # goal_1.pose.orientation.z = 0.0
            goal_1.pose.orientation.w = 0.0
            print(goal_1.header.seq)
            # Might have to edit this . . . 
            plan = planner.plan_to_pose_old(goal_1, [OrientationConstraint])
            input("Press <Enter> to move the right arm to goal pose 1: ")
            if not my_controller.execute_plan(plan[1]): 
                raise Exception("Execution failed")
        except Exception as e:
            print(e)
            traceback.print_exc()
        else:
            break


    # while not rospy.is_shutdown():

    #     while not rospy.is_shutdown():
    #         try:
    #             # x, y, z = 0.8, 0.05, 0.07
    #             goal_1 = sawyer_init
    #             # goal_1.header.frame_id = "base"

    #             # #x, y, and z position
    #             # goal_1.pose.position.x = x
    #             # goal_1.pose.position.y = y
    #             # goal_1.pose.position.z = z

    #             # #Orientation as a quaternion
    #             # goal_1.pose.orientation.x = 0.0
    #             # goal_1.pose.orientation.y = -1.0
    #             # goal_1.pose.orientation.z = 0.0
    #             # goal_1.pose.orientation.w = 0.0

    #             # Might have to edit this . . . 
    #             plan = planner.plan_to_pose(goal_1, [OrientationConstraint])
    #             input("Press <Enter> to move the right arm to goal pose 1: ")
    #             if not my_controller.execute_plan(plan[1]): 
    #                 raise Exception("Execution failed")
    #         except Exception as e:
    #             print(e)
    #             traceback.print_exc()
    #         else:
    #             break

    #     while not rospy.is_shutdown():
    #         try:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:

    #             # UPDATE VISION RAY OBSTACLES:
    #             # Obstacle vectors relative to head_camera frame:
    #             tag1_diff = m1_pos - translation_w
    #             tag2_diff = m2_pos - translation_w
    #             tag3_diff = m3_pos - translation_w
    #             tag4_diff = m4_pos - translation_w

    #             # Distances of ar_tags from camera
    #             m1_dist = np.sqrt(np.sum(tag1_diff))
    #             m2_dist = np.sqrt(np.sum(tag2_diff))
    #             m3_dist = np.sqrt(np.sum(tag3_diff))
    #             m4_dist = np.sqrt(np.sum(tag4_diff)) 

    #             # cog of obstacle rectangles relative to base frame:
    #             cog_el1 = m1_dist/2*(m1_pos + translation_w)
    #             cog_el2 = m2_dist/2*(m2_pos + translation_w)
    #             cog_el3 = m3_dist/2*(m3_pos + translation_w)
    #             cog_el4 = m4_dist/2*(m4_pos + translation_w)

    #             # Vision obstacle sizes:
    #             # IS THIS THE RIGHT ORDER????
    #             size_array_v1 = np.array([m1_dist, 0.01, 0.01])
    #             size_array_v2 = np.array([m2_dist, 0.01, 0.01])
    #             size_array_v3 = np.array([m3_dist, 0.01, 0.01])
    #             size_array_v4 = np.array([m4_dist, 0.01, 0.01])
                
    #             # Needed: position/orientation of cogs, and size of obstacle

    #             # RAY 1
    #             co_vi_1 = CollisionObject()
    #             co_vi_1.operation = CollisionObject.ADD
    #             co_vi_1.id = 'vision_obstacle_1'
    #             co_vi_1.header = 'vision_obstacle_1'

    #             # Create a box primitive, which will be inside the CollisionObject
    #             vi1 = SolidPrimitive()
    #             vi1.type = SolidPrimitive.BOX
    #             vi1.dimensions = size_array_v1

    #             # Fill the collision object with primitive(s)
    #             co_vi_1.primitives = [vi1]
    #             vi1p = PoseStamped()
    #             m1_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_1', rospy.Time()).transform.rotation
    #             vi1p.pose.position = cog_el1
    #             vi1p.pose.orientation = m1_vec_quat
    #             planner.add_box_obstacle(size_array_v1, 'vision_obstacle_1', vi1p.pose)

    #             # RAY 2
    #             co_vi_2 = CollisionObject()
    #             co_vi_2.operation = CollisionObject.ADD
    #             co_vi_2.id = 'vision_obstacle_2'
    #             co_vi_2.header = 'vision_obstacle_2'

    #             # Create a box primitive, which will be inside the CollisionObject
    #             vi2 = SolidPrimitive()
    #             vi2.type = SolidPrimitive.BOX
    #             vi2.dimensions = size_array_v2

    #             # Fill the collision object with primitive(s)
    #             co_vi_2.primitives = [vi2]
    #             vi2p = PoseStamped()
    #             m2_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_2', rospy.Time()).transform.rotation
    #             vi2p.pose.position = cog_el2
    #             vi2p.pose.orientation = m2_vec_quat
    #             planner.add_box_obstacle(size_array_v2, 'vision_obstacle_2', vi2p.pose)

    #             # RAY 3 
    #             co_vi_3 = CollisionObject()
    #             co_vi_3.operation = CollisionObject.ADD
    #             co_vi_3.id = 'vision_obstacle_3'
    #             co_vi_3.header = 'vision_obstacle_3'

    #             # Create a box primitive, which will be inside the CollisionObject
    #             vi3 = SolidPrimitive()
    #             vi3.type = SolidPrimitive.BOX
    #             vi3.dimensions = size_array_v3

    #             # Fill the collision object with primitive(s)
    #             co_vi_3.primitives = [vi3]
    #             vi3p = PoseStamped()
    #             m3_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_3', rospy.Time()).transform.rotation
    #             vi3p.pose.position = cog_el3
    #             vi3p.pose.orientation = m3_vec_quat
    #             planner.add_box_obstacle(size_array_v3, 'vision_obstacle_3', vi3p.pose)

    #             # RAY 4
    #             co_vi_4 = CollisionObject()
    #             co_vi_4.operation = CollisionObject.ADD
    #             co_vi_4.id = 'vision_obstacle_4'
    #             co_vi_4.header = 'vision_obstacle_4'

    #             # Create a box primitive, which will be inside the CollisionObject
    #             vi4 = SolidPrimitive()
    #             vi4.type = SolidPrimitive.BOX
    #             vi4.dimensions = size_array_v4

    #             # Fill the collision object with primitive(s)
    #             co_vi_4.primitives = [vi4]
    #             vi4p = PoseStamped()
    #             m4_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_4', rospy.Time()).transform.rotation
    #             vi4p.pose.position = cog_el4
    #             vi4p.pose.orientation = m4_vec_quat
    #             planner.add_box_obstacle(size_array_v4, 'vision_obstacle_4', vi4p.pose)



    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:



    #             goal_2 = PoseStamped()
    #             goal_2.header.frame_id = "base"

    #             #x, y, and z position
    #             goal_2.pose.position.x = 0.6
    #             goal_2.pose.position.y = -0.3
    #             goal_2.pose.position.z = 0.0

    #             #Orientation as a quaternion
    #             goal_2.pose.orientation.x = 0.0
    #             goal_2.pose.orientation.y = -1.0
    #             goal_2.pose.orientation.z = 0.0
    #             goal_2.pose.orientation.w = 0.0

    #             plan = planner.plan_to_pose(goal_2, [])
    #             input("Press <Enter> to move the right arm to goal pose 2: ")
    #             if not my_controller.execute_plan(plan[1]):
    #                 raise Exception("Execution failed")
    #         except Exception as e:
    #             print(e)
    #         else:
    #             break

    #     while not rospy.is_shutdown():
    #         try:
    #             goal_3 = PoseStamped()
    #             goal_3.header.frame_id = "base"

# if not rospy.is_shutdown():

    #     while not rospy.is_shutdown():
    #         try:
    #             # x, y, z = 0.8, 0.05, 0.07
    #             goal_1 = sawyer_init
    #             # goal_1.header.frame_id = "base"

    #             # #x, y, and z position
    #             # goal_1.pose.position.x = x
    #             # goal_1.pose.position.y = y
    #             # goal_1.pose.position.z = z

    #             # #Orientation as a quaternion
    #             # goal_1.pose.orientation.x = 0.0
    #             # goal_1.pose.orientation.y = -1.0
    #             # goal_1.pose.orientation.z = 0.0
    #             # goal_1.pose.orientation.w = 0.0

    #             # Might have to edit this . . . 
    #             plan = planner.plan_to_pose(goal_1, [OrientationConstraint])
    #             input("Press <Enter> to move the right arm to goal pose 1: ")
    #             if not my_controller.execute_plan(plan[1]): 
    #                 raise Exception("Execution failed")
    #         except Exception as e:
    #             print(e)
    #             traceback.print_exc()
    #         else:
    #             break

    #     while not rospy.is_shutdown():
    #         try:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:

    #             # UPDATE VISION RAY OBSTACLES:
    #             # Obstacle vectors relative to head_camera frame:
    #             tag1_diff = m1_pos - translation_w
    #             tag2_diff = m2_pos - translation_w
    #             tag3_diff = m3_pos - translation_w
    #             tag4_diff = m4_pos - translation_w

    #             # Distances of ar_tags from camera
    #             m1_dist = np.sqrt(np.sum(tag1_diff))
    #             m2_dist = np.sqrt(np.sum(tag2_diff))
    #             m3_dist = np.sqrt(np.sum(tag3_diff))
    #             m4_dist = np.sqrt(np.sum(tag4_diff)) 

    #             # cog of obstacle rectangles relative to base frame:
    #             cog_el1 = m1_dist/2*(m1_pos + translation_w)
    #             cog_el2 = m2_dist/2*(m2_pos + translation_w)
    #             cog_el3 = m3_dist/2*(m3_pos + translation_w)
    #             cog_el4 = m4_dist/2*(m4_pos + translation_w)

    #             # Vision obstacle sizes:
    #             # IS THIS THE RIGHT ORDER????
    #             size_array_v1 = np.array([m1_dist, 0.01, 0.01])
    #             size_array_v2 = np.array([m2_dist, 0.01, 0.01])
    #             size_array_v3 = np.array([m3_dist, 0.01, 0.01])
    #             size_array_v4 = np.array([m4_dist, 0.01, 0.01])
                
    #             # Needed: position/orientation of cogs, and size of obstacle

    #             # RAY 1
    #             co_vi_1 = CollisionObject()
    #             co_vi_1.operation = CollisionObject.ADD
    #             co_vi_1.id = 'vision_obstacle_1'
    #             co_vi_1.header = 'vision_obstacle_1'

    #             # Create a box primitive, which will be inside the CollisionObject
    #             vi1 = SolidPrimitive()
    #             vi1.type = SolidPrimitive.BOX
    #             vi1.dimensions = size_array_v1

    #             # Fill the collision object with primitive(s)
    #             co_vi_1.primitives = [vi1]
    #             vi1p = PoseStamped()
    #             m1_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_1', rospy.Time()).transform.rotation
    #             vi1p.pose.position = cog_el1
    #             vi1p.pose.orientation = m1_vec_quat
    #             planner.add_box_obstacle(size_array_v1, 'vision_obstacle_1', vi1p.pose)

    #             # RAY 2
    #             co_vi_2 = CollisionObject()
    #             co_vi_2.operation = CollisionObject.ADD
    #             co_vi_2.id = 'vision_obstacle_2'
    #             co_vi_2.header = 'vision_obstacle_2'

    #             # Create a box primitive, which will be inside the CollisionObject
    #             vi2 = SolidPrimitive()
    #             vi2.type = SolidPrimitive.BOX
    #             vi2.dimensions = size_array_v2

    #             # Fill the collision object with primitive(s)
    #             co_vi_2.primitives = [vi2]
    #             vi2p = PoseStamped()
    #             m2_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_2', rospy.Time()).transform.rotation
    #             vi2p.pose.position = cog_el2
    #             vi2p.pose.orientation = m2_vec_quat
    #             planner.add_box_obstacle(size_array_v2, 'vision_obstacle_2', vi2p.pose)

    #             # RAY 3 
    #             co_vi_3 = CollisionObject()
    #             co_vi_3.operation = CollisionObject.ADD
    #             co_vi_3.id = 'vision_obstacle_3'
    #             co_vi_3.header = 'vision_obstacle_3'

    #             # Create a box primitive, which will be inside the CollisionObject
    #             vi3 = SolidPrimitive()
    #             vi3.type = SolidPrimitive.BOX
    #             vi3.dimensions = size_array_v3

    #             # Fill the collision object with primitive(s)
    #             co_vi_3.primitives = [vi3]
    #             vi3p = PoseStamped()
    #             m3_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_3', rospy.Time()).transform.rotation
    #             vi3p.pose.position = cog_el3
    #             vi3p.pose.orientation = m3_vec_quat
    #             planner.add_box_obstacle(size_array_v3, 'vision_obstacle_3', vi3p.pose)

    #             # RAY 4
    #             co_vi_4 = CollisionObject()
    #             co_vi_4.operation = CollisionObject.ADD
    #             co_vi_4.id = 'vision_obstacle_4'
    #             co_vi_4.header = 'vision_obstacle_4'

    #             # Create a box primitive, which will be inside the CollisionObject
    #             vi4 = SolidPrimitive()
    #             vi4.type = SolidPrimitive.BOX
    #             vi4.dimensions = size_array_v4

    #             # Fill the collision object with primitive(s)
    #             co_vi_4.primitives = [vi4]
    #             vi4p = PoseStamped()
    #             m4_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_4', rospy.Time()).transform.rotation
    #             vi4p.pose.position = cog_el4
    #             vi4p.pose.orientation = m4_vec_quat
    #             planner.add_box_obstacle(size_array_v4, 'vision_obstacle_4', vi4p.pose)



    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:
    #             # REAL TIME CONTROLLER:



    #             goal_2 = PoseStamped()
    #             goal_2.header.frame_id = "base"

    #             #x, y, and z position
    #             goal_2.pose.position.x = 0.6
    #             goal_2.pose.position.y = -0.3
    #             goal_2.pose.position.z = 0.0

    #             #Orientation as a quaternion
    #             goal_2.pose.orientation.x = 0.0
    #             goal_2.pose.orientation.y = -1.0
    #             goal_2.pose.orientation.z = 0.0
    #             goal_2.pose.orientation.w = 0.0

    #             plan = planner.plan_to_pose(goal_2, [])
    #             input("Press <Enter> to move the right arm to goal pose 2: ")
    #             if not my_controller.execute_plan(plan[1]):
    #                 raise Exception("Execution failed")
    #         except Exception as e:
    #             print(e)
    #         else:
    #             break

    #     while not rospy.is_shutdown():
    #         try:
    #             goal_3 = PoseStamped()
    #             goal_3.header.frame_id = "base"

    #             #x, y, and z position
    #             goal_3.pose.position.x = 0.6
    #             goal_3.pose.position.y = -0.1
    #             goal_3.pose.position.z = 0.1

    #             #Orientation as a quaternion
    #             goal_3.pose.orientation.x = 0.0
    #             goal_3.pose.orientation.y = -1.0
    #             goal_3.pose.orientation.z = 0.0
    #             goal_3.pose.orientation.w = 0.0

    #             plan = planner.plan_to_pose(goal_3, [])
    #             input("Press <Enter> to move the right arm to goal pose 3: ")
    #             if not my_controller.execute_plan(plan[1]):
    #                 raise Exception("Execution failed")
    #         except Exception as e:
    #             print(e)
    #         else:
    #             break    #             #x, y, and z position
    #             goal_3.pose.position.x = 0.6
    #             goal_3.pose.position.y = -0.1
    #             goal_3.pose.position.z = 0.1

    #             #Orientation as a quaternion
    #             goal_3.pose.orientation.x = 0.0
    #             goal_3.pose.orientation.y = -1.0
    #             goal_3.pose.orientation.z = 0.0
    #             goal_3.pose.orientation.w = 0.0

    #             plan = planner.plan_to_pose(goal_3, [])
    #             input("Press <Enter> to move the right arm to goal pose 3: ")
    #             if not my_controller.execute_plan(plan[1]):
    #                 raise Exception("Execution failed")
    #         except Exception as e:
    #             print(e)
    #         else:
    #             break


def exec_plan(r):
    while not rospy.is_shutdown():
        # create a subscriber to \tag_info
        # runs the callback function 'execute_plan' with the input from the /sawyer_cog topic
        rospy.Subscriber('/tag_info', VisualData, execute_plan) # tag_info
        r.sleep()


if __name__ == '__main__':
    # Run this program as a new node in the ROS computation graph called /sawyer_vision.
    rospy.init_node('moveit_node')
    r = rospy.Rate(1)

    # Check if the node has received a signal to shut down
    # If not, run the talker method
    try:
        print ('planner node is Running')
        exec_plan(r)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node('moveit_node')
#     main()
