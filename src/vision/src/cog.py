#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import numpy as np


from geometry_msgs.msg import Pose # pose data of end effector
from vision.msg import VisualData
from vision.msg import SawyerCog


pub = rospy.Publisher('/waypoint', VisualData, queue_size=10)

def callback(message):
    while not rospy.is_shutdown():
        try:
            print("reading vision node")
            hand_pos = (message.human_ar.pose.position.x, message.human_ar.pose.position.y, message.human_ar.pose.position.z)
            m1_pos = (message.m1.pose.position.x, message.m1.pose.position.y, message.m1.pose.position.z)
            m2_pos = (message.m2.pose.position.x, message.m2.pose.position.y, message.m2.pose.position.z)
            m3_pos = (message.m3.pose.position.x, message.m3.pose.position.y, message.m3.pose.position.z)
            m4_pos = (message.m4.pose.position.x, message.m4.pose.position.y, message.m4.pose.position.z)

            dist = m2_pos[0] - hand_pos[0]
            sawyer_x_cog = m4_pos[0] + dist
            sawyer_x_init = (m4_pos[0] + m3_pos[0]) / 2
            sawyer_y = m2_pos[1] + message.obj_length
            sawyer_z = m2_pos[2]

            sawyer_cog = Pose()
            sawyer_cog.pos.position.x = sawyer_x_cog
            sawyer_cog.pos.position.y = sawyer_y
            sawyer_cog.pos.position.z = sawyer_z
            
            sawyer_init = Pose()
            sawyer_init.pos.position.x = sawyer_x_init
            sawyer_init.pos.position.y = sawyer_y
            sawyer_init.pos.position.z = sawyer_z


            pub.publish(SawyerCog(sawyer_cog, sawyer_init))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          pass

def get_cog():
     rospy.Subscriber("/tag_info", VisualData, callback)
     rospy.spin()


if __name__ == '__main__':
  # Run this program as a new node in the ROS computation graph called /sawyer_vision.
  rospy.init_node('sawyer_cog', anonymous=True)
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  try:
    print ('Sawyer cog is Running')
    get_cog()
  except rospy.ROSInterruptException:
    pass
