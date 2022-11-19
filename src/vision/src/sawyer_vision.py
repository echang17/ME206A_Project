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

import intera_interface
from intera_interface import Limb

from geometry_msgs.msg import PoseStamped # pose data of joints/markers
from geometry_msgs.msg import TransformStamped # transform data between joints/markers
from vision.msg import VisualData # not sure if this is right?

#Define the method which contains the main functionality of the node.
def marker_locator():
  """
  Calculates object dimensions and pose in space, in addition to
  pose of human operator hand
  Markers 1 & 2 or 3 & 4 define the length of the object
  Markers 1 & 4 or 2 & 3 define the width of the object
  Assume human stands to the side of the robot (instead of opposite the table)
  and robot lifts with gripper in a configuration parallel to own body

  """
    # create a publisher to publish object pose, human pose, and where Sawyer should position gripper to lift
    pub = rospy.Publisher('/TOPICNAME', VisualData, queue_size=10) # left queue_size=10 from lab2

    # tf Listener to get transforms between markers and robot
    tfBuffer = tf2_ros.Buffer() # store buffer of previous transforms
    tfListener = tf2_ros.TransformListener(tfBuffer) #subscribe to tf topic and maintain tf graph inside Buffer

    while not rospy.is_shutdown():
        try:
            # Object Markers:
            # Marker 1 (id 15)
            m1 = PoseStamped()
            m1.header.stamp = rospy.Time.now()
            m1.header.frame_id = "ar_marker_15"
            m1x = m1.pose.position.x # x axis to human operator's right
            m1y = m1.pose.position.y # y axis to human operator's forward
            m1z = m1.pose.position.z # z axis points up

            # Marker 2 (id 4)
            m2 = PoseStamped()
            m2.header.stamp = rospy.Time.now()
            m2.header.frame_id = "ar_marker_4"
            m2x = m2.pose.position.x
            m2y = m2.pose.position.y
            m2z = m2.pose.position.z

            # Marker 3 (id 17)- redundant for now until markers obscured
            m3 = PoseStamped()
            m3.header.stamp = rospy.Time.now()
            m3.header.frame_id = "ar_marker_17"
            m3x = m3.pose.position.x
            m3y = m3.pose.position.y
            m3z = m3.pose.position.z

            # Marker 4 (id 13)
            m4 = PoseStamped()
            m4.header.stamp = rospy.Time.now()
            m4.header.frame_id = "ar_marker_13"
            m4x = m4.pose.position.x
            m4y = m4.pose.position.y
            m4z = m4.pose.position.z

            # object shape: distances between pairs of markers
            # add extra code if a marker is obscured?
            obj_length = np.sqrt((m1x - m2x)**2 + (m1y - m2y)**2 + (m1z - m2z)**2)
            obj_width = np.sqrt((m1x - m4x)**2 (m1y - m4y)**2 + (m1z - m4z)**2)

            # coordinates of where to lift: center of width between markers on side by markers 1 & 4
            lift_location = PoseStamped()
            lift_location.header.stamp = rospy.Time.now()
            lift_location.header.frame_id = "right_gripper"
            lift_location.pose.position.x = m4x
            lift_location.pose.position.y = m4y - obj_width/2
            lift_location.pose.position.z = m4z
            lift_location.pose.orientation.x = -1 # no rotation (hold flat?- check these values)
            lift_location.pose.orientation.y = 0
            lift_location.pose.orientation.z = 0
            lift_location.pose.orientation.w = 0 

            # human AR tag location: **TODO**

            pub.publish(VisualData (obj_length,obj_width,lift_location,m1,m2,m3,m4,human_ar))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /sawyer_vision.
  rospy.init_node('sawyer_vision', anonymous=True)

  try:
    marker_locator()
  except rospy.ROSInterruptException:
    pass