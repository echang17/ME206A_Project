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
			load.pose.position.x = sawyer_x_init
			load.pose.position.y = sawyer_y
			load.pose.position.z = sawyer_z - underside_offset
			load.pose.orientation.x = sawyer_init.orientation.x
			load.pose.orientation.y = sawyer_init.orientation.y
			load.pose.orientation.z = sawyer_init.orientation.z
			load.pose.orientation.w = sawyer_init.orientation.w
			
			size_array = np.array([width, length, 0.25])
			# box_obstacle_params = {'Size': size_array, 'name': 'load', 'object': load}
			# box_obstacle_params = [size_array, 'table', table]
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