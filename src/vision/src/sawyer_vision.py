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

# import intera_interface
# from intera_interface import Limb

from geometry_msgs.msg import PoseStamped # pose data of joints/markers
from geometry_msgs.msg import TransformStamped # transform data between joints/markers
from vision.msg import VisualData
from ar_track_alvar_msgs.msg import AlvarMarker # built-in message type for 1 AR tag
from ar_track_alvar_msgs.msg import AlvarMarkers # built-in message type for multiple AR tags

 
  # This node:
  # 1. Subscribes to /ar_pose_marker to get real-time data of all AR tag locations
  # 2. Calculates object dimensions and pose in space, in addition to
  #   pose of human operator hand
  # 3. Publishes these real-time AR tag locations of the object and the human collaborator 
  #   to a topic called 'tag_info'

  # Other notes: 
  # Markers 1 & 4 or 2 & 3 define the length of the object
  # Markers 1 & 2 or 3 & 4 define the width of the object
  # Assume human stands to the side of the robot (instead of opposite the table)
  # and robot lifts with gripper in a configuration parallel to own body


# create a publisher to publish object pose, human pose, and where Sawyer should 
# position gripper to lift to a topic called 'tag_info'
pub = rospy.Publisher('tag_info', VisualData, queue_size=10) # left queue_size=10 from lab2

# define a callback method which is called whenever this node receives a 
# message on its subscribed topic. received message is passes as argument to callback()
def callback(message):
  while not rospy.is_shutdown():
    try:
      # read and save AR tag information from /ar_pose_marker topic
      # print('number of markers detected:')
      # print(len(message.markers))
      avail_ids = []

      for i in range(len(message.markers)): # loop through all markers detected
        if message.markers[i].id == 1: # ar_marker_1
          m1 = PoseStamped()
          m1.header = message.markers[i].header
          m1.pose = message.markers[i].pose.pose

          avail_ids.append(1)
          m1x = m1.pose.position.x # x axis to human operator's right
          m1y = m1.pose.position.y # y axis to human operator's forward
          m1z = m1.pose.position.z # z axis points up
        elif message.markers[i].id == 2: # ar_marker_2
          m2 = PoseStamped()
          m2.header = message.markers[i].header
          m2.pose = message.markers[i].pose.pose

          avail_ids.append(2)
          m2x = m2.pose.position.x # x axis to human operator's right
          m2y = m2.pose.position.y # y axis to human operator's forward
          m2z = m2.pose.position.z # z axis points up
        elif message.markers[i].id == 3: # ar_marker_3
          m3 = PoseStamped()
          m3.header = message.markers[i].header
          m3.pose = message.markers[i].pose.pose

          avail_ids.append(3)
          m3x = m3.pose.position.x # x axis to human operator's right
          m3y = m3.pose.position.y # y axis to human operator's forward
          m3z = m3.pose.position.z # z axis points up
        elif message.markers[i].id == 4: # ar_marker_4
          m4 = PoseStamped()
          m4.header = message.markers[i].header
          m4.pose = message.markers[i].pose.pose

          avail_ids.append(4)
          m4x = m4.pose.position.x # x axis to human operator's right
          m4y = m4.pose.position.y # y axis to human operator's forward
          m4z = m4.pose.position.z # z axis points up
        else: # marker id 4 = ar_marker_5
          human_ar = PoseStamped()
          human_ar.header = message.markers[i].header
          human_ar.pose = message.markers[i].pose.pose
      
      # print('available object marker ids:')
      # print(avail_ids)
      #object shape: distances between pairs of markers

      # length: need both 1&4 opr 2&3
      # width: need both 1&2 or 3&4

      if 1 in avail_ids and 4 in avail_ids:
        obj_length = obj_length = np.sqrt((m1x - m4x)**2 + (m1y - m4y)**2 + (m1z - m4z)**2)
      elif 2 in avail_ids and 3 in avail_ids:
        obj_length = np.sqrt((m2x - m3x)**2 + (m2y - m3y)**2 + (m2z - m3z)**2)
      else:
        print('not enough markers detected to determine length')
        obj_length = 0
      
      if 1 in avail_ids and 2 in avail_ids:
        obj_width = np.sqrt((m1x - m2x)**2 + (m1y - m2y)**2 + (m1z - m2z)**2)
      elif 3 in avail_ids and 4 in avail_ids:
         obj_width = np.sqrt((m3x - m4x)**2 + (m3y - m4y)**2 + (m3z - m4z)**2)
      else:
        print('not enough markers detected to determine width')
        obj_width = 0
      
      pub.publish(VisualData (obj_length,obj_width,m1,m2,m3,m4,human_ar))
      print('publishing to /tag_info')
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass

# define the method which contains the main functionality of the node
def marker_locator():
  # create a subscriber to subscribe to /ar_pose_marker to get marker locations
  rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback)
  rospy.spin()



# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Run this program as a new node in the ROS computation graph called /sawyer_vision.
  rospy.init_node('sawyer_vision', anonymous=True)

  # Check if the node has received a signal to shut down
  # If not, run the talker method
  try:
    print ('Sawyer Vision is Running')
    marker_locator()
  except rospy.ROSInterruptException:
    pass


# # OLD BELOW
# #Define the method which contains the main functionality of the node
# def marker_locator()
#   """
#   Publishes real-time AR tag locations of the object and the human collaborator to a topic called 'tag_info'

#   Calculates object dimensions and pose in space, in addition to
#   pose of human operator hand
#   Markers 1 & 2 or 3 & 4 define the length of the object
#   Markers 1 & 4 or 2 & 3 define the width of the object
#   Assume human stands to the side of the robot (instead of opposite the table)
#   and robot lifts with gripper in a configuration parallel to own body

#   """
#     # create a publisher to publish object pose, human pose, and where Sawyer should position gripper to lift
#     # to a topic called 'tag_info'
#     pub = rospy.Publisher('tag_info', VisualData, queue_size=10) # left queue_size=10 from lab2

#     # # tf Listener to get transforms between markers and robot
#     # tfBuffer = tf2_ros.Buffer() # store buffer of previous transforms
#     # tfListener = tf2_ros.TransformListener(tfBuffer) #subscribe to tf topic and maintain tf graph inside Buffer

#     while not rospy.is_shutdown():
#         try:
#             # Object Markers:
#             # Marker 1 (id 1)
#             m1 = PoseStamped()
#             m1.header.stamp = rospy.Time.now()
#             m1.header.frame_id = "ar_marker_1"
#             m1x = m1.pose.position.x # x axis to human operator's right
#             m1y = m1.pose.position.y # y axis to human operator's forward
#             m1z = m1.pose.position.z # z axis points up

#             # Marker 2 (id 2)
#             m2 = PoseStamped()
#             m2.header.stamp = rospy.Time.now()
#             m2.header.frame_id = "ar_marker_2"
#             m2x = m2.pose.position.x
#             m2y = m2.pose.position.y
#             m2z = m2.pose.position.z

#             # Marker 3 (id 3)- redundant for now until markers obscured
#             m3 = PoseStamped()
#             m3.header.stamp = rospy.Time.now()
#             m3.header.frame_id = "ar_marker_3"
#             m3x = m3.pose.position.x
#             m3y = m3.pose.position.y
#             m3z = m3.pose.position.z

#             # Marker 4 (id 4)
#             m4 = PoseStamped()
#             m4.header.stamp = rospy.Time.now()
#             m4.header.frame_id = "ar_marker_4"
#             m4x = m4.pose.position.x
#             m4y = m4.pose.position.y
#             m4z = m4.pose.position.z

#             # object shape: distances between pairs of markers
#             # add extra code if a marker is obscured?
#             obj_length = np.sqrt((m1x - m2x)**2 + (m1y - m2y)**2 + (m1z - m2z)**2)
#             obj_width = np.sqrt((m1x - m4x)**2 + (m1y - m4y)**2 + (m1z - m4z)**2)

#             # coordinates of where to lift: center of width between markers on side by markers 1 & 4
#             lift_location = PoseStamped()
#             lift_location.header.stamp = rospy.Time.now()
#             lift_location.header.frame_id = "right_gripper"
#             lift_location.pose.position.x = m4x
#             lift_location.pose.position.y = m4y - obj_width/2
#             lift_location.pose.position.z = m4z
#             lift_location.pose.orientation.x = -1 # no rotation (hold flat?- check these values)
#             lift_location.pose.orientation.y = 0
#             lift_location.pose.orientation.z = 0
#             lift_location.pose.orientation.w = 0 

#             # human AR tag location (id 5)
#             human_ar = PoseStamped()
#             human_ar.header.stamp = rospy.Time.now()
#             human_ar.header.frame_id = "ar_marker_5"
#             human_arx = human_ar.pose.position.x
#             human_ary = human_ar.pose.position.y
#             human_arz = human_ar.pose.position.z


#             pub.publish(VisualData (obj_length,obj_width,lift_location,m1,m2,m3,m4,human_ar))
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             pass
      
# # This is Python's sytax for a main() method, which is run by default
# # when exectued in the shell
# if __name__ == '__main__':
#   # Check if the node has received a signal to shut down
#   # If not, run the talker method

#   #Run this program as a new node in the ROS computation graph 
#   #called /sawyer_vision.
#   rospy.init_node('sawyer_vision', anonymous=True)

#   try:
#     marker_locator()
#   except rospy.ROSInterruptException:
#     pass