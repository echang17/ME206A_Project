#!/usr/bin/env python
"""
Create node that publishes obstacle object representing 
the box/object to be lifted. This object allows planning a safe path 
to make contact with the underside of the object

Author: Aaron Kandel
"""
# import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
import tf.transformations as tr
from ar_track_alvar_msgs.msg import AlvarMarker # built-in message type for 1 AR tag
from ar_track_alvar_msgs.msg import AlvarMarkers # built-in message type for multiple AR tags



from vision.msg import VisualData
from vision.msg import SawyerCog
from path_planner import PathPlanner
import matplotlib.pyplot as plt

import pickle
import os


def pub_obst(Visual_Data):
	"""
    Adds a rectangular prism obstacle to the planning scene, representing
    the object we seek to collaboratively move with the sawyer robot.

    Also adds collision obstacles in the line of sight of the ar tags from
    the camera, so the robot does not obstruct its own vision.

    Inputs:
    Visual_Data: Visual_Data message from tag_info topic
    VisualData includes the following variables:
	float64 obj_length
	float64 obj_width
	geometry_msgs/PoseStamped m1
	geometry_msgs/PoseStamped m2
	geometry_msgs/PoseStamped m3
	geometry_msgs/PoseStamped m4
	geometry_msgs/PoseStamped human_ar
    """  
	while not rospy.is_shutdown():
		try:
			width = Visual_Data.obj_width
			length = Visual_Data.obj_length

			el1 = Visual_Data.m1
			el2 = Visual_Data.m2
			el3 = Visual_Data.m3
			el4 = Visual_Data.m4

			m1_pos = (el1.pose.position.x, el1.pose.position.y, el1.pose.position.z)
            m2_pos = (el2.pose.position.x, el2.pose.position.y, el2.pose.position.z)
            m3_pos = (el3.pose.position.x, el3.pose.position.y, el3.pose.position.z)
            m4_pos = (el4.pose.position.x, el4.pose.position.y, el4.pose.position.z)

            dist = m2_pos[0] - hand_pos[0]
            sawyer_x_cog = m4_pos[0] + dist
            sawyer_x_init = (m4_pos[0] + m3_pos[0]) / 2
            sawyer_y = m2_pos[1] + message.obj_length
            sawyer_z = m2_pos[2]

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
            
            sawyer_init.orientation.x = message.m1.pose.orientation.x
            sawyer_init.orientation.y = message.m1.pose.orientation.y#-1.0 * message.human_ar.pose.orientation.y
            sawyer_init.orientation.z = message.m1.pose.orientation.z
            sawyer_init.orientation.w = message.m1.pose.orientation.w
			

			# # Human AR Tag:
			# h = Visual_Data.human_ar

			# load object
			load = PoseStamped()
			load.pose.position = sawyer_cog.position
			load.pose.orientation = sawyer_init.orientation
			# load.pose.position.x = sawyer_x_init
			# load.pose.position.y = sawyer_y
			# load.pose.position.z = sawyer_z #- underside_offset
			# load.pose.orientation.x = sawyer_init.orientation.x
			# load.pose.orientation.y = sawyer_init.orientation.y
			# load.pose.orientation.z = sawyer_init.orientation.z
			# load.pose.orientation.w = sawyer_init.orientation.w
			
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

	        # Publish the object
	        psp = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)
	        psp.publish(co)


	        ############################
	        # Line-of-sight obstacles: #
	        ############################

	        # Camera Pose:
	        transform = tfBuffer.lookup_transform('reference/base', 'head_camera', rospy.Time()).transform  
	        translation = transform.translation
	        translation_w = [translation.x, translation.y, translation.z]
	        rot = transform.rotation
	        cam_loc = PoseStamped()
	        cam_loc.pose.orientation = rot
	        cam_loc.pose.position = translation
	        # cam_trans_w = [cam_loc.pose.position.x]

	        
	        # el1_position = el1.pose.position.x
	        # el2_pose = el2.pose
	        # el3_pose = el3.pose
	        # el4_pose = el4.pose

	        # m1d = tfBuffer.lookup_transform('head_camera', 'ar_marker_1', rospy.Time()).transform.translation
	        # m1dw = [m1d.x, m1d.y, m1d.z]
	        # m1_dist = np.sqrt(m1d.x**2 + m1dw.y**2 + m1dw.z**2)

	        # m1d = tfBuffer.lookup_transform('head_camera', 'ar_marker_1', rospy.Time()).transform.translation
	        # m1dw = [m1d.x, m1d.y, m1d.z]
	        # m1_dist = np.sqrt(m1d.x**2 + m1dw.y**2 + m1dw.z**2)

	        # Obstacle vectors relative to head_camera frame:
	        tag1_diff = m1_pos - translation_w
	        tag2_diff = m2_pos - translation_w
	        tag3_diff = m3_pos - translation_w
	        tag4_diff = m4_pos - translation_w

	        # Distances of ar_tags from camera
	        m1_dist = np.sqrt(np.sum(tag1_diff))
	        m2_dist = np.sqrt(np.sum(tag2_diff))
	        m3_dist = np.sqrt(np.sum(tag3_diff))
	        m4_dist = np.sqrt(np.sum(tag4_diff)) 

	        # cog of obstacle rectangles relative to base frame:
	        cog_el1 = m1_dist/2*(m1_pos + translation_w)
	        cog_el2 = m2_dist/2*(m2_pos + translation_w)
	        cog_el3 = m3_dist/2*(m3_pos + translation_w)
	        cog_el4 = m4_dist/2*(m4_pos + translation_w)

	        # Vision obstacle sizes:
	        # IS THIS THE RIGHT ORDER????
	        size_array_v1 = np.array([m1_dist, 0.01, 0.01])
	        size_array_v2 = np.array([m2_dist, 0.01, 0.01])
	        size_array_v3 = np.array([m3_dist, 0.01, 0.01])
	        size_array_v4 = np.array([m4_dist, 0.01, 0.01])
	        
	        # Needed: position/orientation of cogs, and size of obstacle

	        # RAY 1
	        co_vi_1 = CollisionObject()
	        co_vi_1.operation = CollisionObject.ADD
	        co_vi_1.id = 'vision_obstacle_1'
	        co_vi_1.header = 'vision_obstacle_1'

	        # Create a box primitive, which will be inside the CollisionObject
	        vi1 = SolidPrimitive()
	        vi1.type = SolidPrimitive.BOX
	        vi1.dimensions = size_array_v1

	        # Fill the collision object with primitive(s)
	        co_vi_1.primitives = [vi1]
	        vi1p = PoseStamped()
	        m1_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_1', rospy.Time()).transform.rotation
	        vi1p.pose.position = cog_el1
	        vi1p.pose.orientation = m1_vec_quat
	        co_vi_1.primitive_poses = [vi1p.pose]

	        # Publish the object
	        psp = rospy.Publisher('/vision_ray1', CollisionObject, queue_size=10)
	        psp.publish(co_vi_1)


	        # RAY 2
	        co_vi_2 = CollisionObject()
	        co_vi_2.operation = CollisionObject.ADD
	        co_vi_2.id = 'vision_obstacle_2'
	        co_vi_2.header = 'vision_obstacle_2'

	        # Create a box primitive, which will be inside the CollisionObject
	        vi2 = SolidPrimitive()
	        vi2.type = SolidPrimitive.BOX
	        vi2.dimensions = size_array_v2

	        # Fill the collision object with primitive(s)
	        co_vi_2.primitives = [vi2]
	        vi2p = PoseStamped()
	        m2_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_2', rospy.Time()).transform.rotation
	        vi2p.pose.position = cog_el2
	        vi2p.pose.orientation = m2_vec_quat
	        co_vi_2.primitive_poses = [vi2p.pose]

	        # Publish the object
	        psp = rospy.Publisher('/vision_ray2', CollisionObject, queue_size=10)
	        psp.publish(co_vi_2)

	        # RAY 3 
	        co_vi_3 = CollisionObject()
	        co_vi_3.operation = CollisionObject.ADD
	        co_vi_3.id = 'vision_obstacle_3'
	        co_vi_3.header = 'vision_obstacle_3'

	        # Create a box primitive, which will be inside the CollisionObject
	        vi3 = SolidPrimitive()
	        vi3.type = SolidPrimitive.BOX
	        vi3.dimensions = size_array_v3

	        # Fill the collision object with primitive(s)
	        co_vi_3.primitives = [vi3]
	        vi3p = PoseStamped()
	        m3_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_3', rospy.Time()).transform.rotation
	        vi3p.pose.position = cog_el3
	        vi3p.pose.orientation = m3_vec_quat
	        co_vi_3.primitive_poses = [vi3p.pose]

	        # Publish the object
	        psp = rospy.Publisher('/vision_ray3', CollisionObject, queue_size=10)
	        psp.publish(co_vi_3)

	        # RAY 4
	        co_vi_4 = CollisionObject()
	        co_vi_4.operation = CollisionObject.ADD
	        co_vi_4.id = 'vision_obstacle_4'
	        co_vi_4.header = 'vision_obstacle_4'

	        # Create a box primitive, which will be inside the CollisionObject
	        vi4 = SolidPrimitive()
	        vi4.type = SolidPrimitive.BOX
	        vi4.dimensions = size_array_v4

	        # Fill the collision object with primitive(s)
	        co_vi_4.primitives = [vi4]
	        vi4p = PoseStamped()
	        m4_vec_quat = tfBuffer.lookup_transform('head_camera', 'ar_marker_4', rospy.Time()).transform.rotation
	        vi4p.pose.position = cog_el4
	        vi4p.pose.orientation = m4_vec_quat
	        co_vi_4.primitive_poses = [vi4p.pose]

	        # Publish the object
	        psp = rospy.Publisher('/vision_ray4', CollisionObject, queue_size=10)
	        psp.publish(co_vi_4)









def obj_data():
    # create a subscriber to \tag_info
    # runs the callback function 'pub_obst' with the input from the /tag_info topic
    rospy.Subscriber('/tag_info', tag_info, pub_obst) # tag_info
    rospy.spin()

if __name__ == '__main__':
	# Run this program as a new node in the ROS computation graph called /sawyer_vision.
	rospy.init_node('collision_object', anonymous=True)
	# Check if the node has received a signal to shut down
	# If not, run the talker method
	try:
		print ('Coll obj node is Running')
		obj_data()
	except rospy.ROSInterruptException:
		pass