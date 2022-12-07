#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import time
import rospy
import tf2_ros
import sys
import numpy as np
import copy
from scipy.spatial.transform import Rotation as R
import sys

from geometry_msgs.msg import PoseStamped # pose data of joints/markers
from geometry_msgs.msg import TransformStamped # transform data between joints/markers
from vision.msg import VisualData
from ar_track_alvar_msgs.msg import AlvarMarker # built-in message type for 1 AR tag
from ar_track_alvar_msgs.msg import AlvarMarkers # built-in message type for multiple AR tags
import tf.transformations as tr

  # This node:
  # * REQUIRES ALL OBJECT MARKERS TO BE VISIBLE TO HEAD CAMERA, OBJECT SHOULD BE FLAT (all markers same z coord)
  # 1. Uses tfBuffer to get each AR tag pose information wrt to reference/base of Sawyer
  #   (gets the transformation between the reference/base and each AR tag) 
  # 2. Interpolates pose of any missing/obscured AR tags
  # 3. Calculates and prints object dimensions

  # Other notes: 
  # Markers 1 & 4 or 2 & 3 define the length of the object
  # Markers 1 & 2 or 3 & 4 define the width of the object
  # Assume human stands to the side of the robot (instead of opposite the table)
  # and robot lifts with gripper in a configuration parallel to own body

# define the vision_calib method which contains the main functionality of the node
def vision_calib():
  r = rospy.Rate(20) #20 Hz - IS THIS NECESSARY?

  while not rospy.is_shutdown():
    try:
      # MARKER 1
      #get TransformStamped message from tfBuffer to get transform between reference/base frame and ar_marker
      transform1 = tfBuffer.lookup_transform('reference/base', 'ar_marker_1', rospy.Time())
      translation1 = transform1.transform.translation # store the translation coordinates
      m1x = translation1.position.x 
      m1y = translation1.position.y

      # MARKER 2
      #get TransformStamped message from tfBuffer to get transform between reference/base frame and ar_marker
      transform2 = tfBuffer.lookup_transform('reference/base', 'ar_marker_2', rospy.Time())
      translation2= transform2.transform.translation # store the translation coordinates
      m2x = translation2.position.x
      m2y = translation2.position.y

      # MARKER 3
      #get TransformStamped message from tfBuffer to get transform between reference/base frame and ar_marker
      transform3 = tfBuffer.lookup_transform('reference/base', 'ar_marker_3', rospy.Time())
      translation3= transform3.transform.translation # store the translation coordinates
      m3x = translation3.position.x
      m3y = translation3.position.y

      # MARKER 4 (technically redundant- don't really need)
      #get TransformStamped message from tfBuffer to get transform between reference/base frame and ar_marker
      transform4 = tfBuffer.lookup_transform('reference/base', 'ar_marker_4', rospy.Time())
      translation4= transform4.transform.translation # store the translation coordinates
      m4x = translation4.position.x
      m4y = translation4.position.y

      # object shape: distances between pairs of markers:
      # length: between 1&4 or 2&3
      # width: between 1&2 or 3&4
      obj_length = abs(m2y - m3y) 
      obj_width = abs(m2x - m1x) 
      print(obj_length)
      print(obj_width)
   
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass

    r.sleep() # IS THIS NECESSARY


# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Run this program as a new node in the ROS computation graph called /sawyer_vision_calib.
  rospy.init_node('sawyer_vision_calib', anonymous=True)

  # create instances of tfBuffer and tfListener
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)

  # Check if the node has received a signal to shut down
  # If not, run the marker_locator method
  try:
    print ('Sawyer Vision Calibration is Running')
    vision_calib()
  except rospy.ROSInterruptException:
    pass
