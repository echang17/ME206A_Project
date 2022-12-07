#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

# import packages
import time
import rospy
import tf2_ros
import sys
import numpy as np
import copy
import sys

from geometry_msgs.msg import PoseStamped # pose data of joints/markers
from geometry_msgs.msg import TransformStamped # transform data between joints/markers
from vision.msg import VisualData
from ar_track_alvar_msgs.msg import AlvarMarker # built-in message type for 1 AR tag
from ar_track_alvar_msgs.msg import AlvarMarkers # built-in message type for multiple AR tags
import tf.transformations as tr

  # This node:
  # 1. Subscribes to /ar_pose_marker to determine which AR tags are visible
  # 2. Uses tfBuffer to get each AR tag pose information wrt to reference/base of Sawyer
  #   (gets the transformation between the reference/base and each AR tag) 
  # 3. Interpolates pose of any missing/obscured AR tags
  # 4. Publishes these real-time AR tag poses on the object and the human collaborator 
  #   to a topic called 'tag_info'

  # Other notes: 
  # Markers 1 & 4 or 2 & 3 define the length of the object
  # Markers 1 & 2 or 3 & 4 define the width of the object
  # Assume human stands to the side of the robot (instead of opposite the table)
  # and robot lifts with gripper in a configuration parallel to own body


# create a publisher to publish object pose, human pose, and where Sawyer should 
# position gripper to lift to a topic called 'tag_info'
pub = rospy.Publisher('/tag_info', VisualData, queue_size=10) # left queue_size=10 from lab2



def getWFPoseStamped(marker_string):
  # function to transform AR tag pose to reference frame
  # input: marker string (ex 'ar_marker_1')
  # output: PoseStamped object for that marker in reference/base frame

  #get TransformStamped message from tfBuffer to get transform between reference/base frame and ar_marker
  t = tfBuffer.lookup_transform('reference/base', marker_string, rospy.Time())
  ts = t.header.stamp # timestamp
  id = t.header.frame_id # transform frame associated with this data (reference/base)
  trans = t.transform.translation # translation/position coordinates
  rot = t.transform.rotation # rotation coordinates
  # create PoseStamped object and give it reference/base transformed pose + other info
  m = PoseStamped() # note that sequence id will be left empty
  m.header.stamp = ts # store corresponding timestamp
  m.header.frame_id = id # store corresponding reference frame name
  m.pose.orientation = rot # store rotation in reference/base frame
  m.pose.position = trans # store translation in reference/base frame

  return m



# define a callback method which is called whenever this node receives a 
# message on its subscribed topic. received message is passed as argument to callback(),
# in addition to length and width values from previous calibration
def callback(message,args):
  # while not rospy.is_shutdown():
  try:
    # Need to check which ids are visible first:
    # read and save AR tag information from /ar_pose_marker topic
    # print('number of markers detected:',len(message.markers))
    allobj_ids = [1,2,3,4] # object ids only
    avail_ids = [] # initialize variable to store visible marker ids
    pose_dict = {} # initialize dictionary to store visible marker ids and their pose
    z_dict = {} # initialize dictionary to store visible marker ids and their z coordinates
    obj_ref_marker = -1 # initialized reference marker number as -1 to check if no object markers visible (for marker interpolation)
    hum_ref_marker = -1 # initialized reference marker number as -1 to check if human_ar marker is visible

    # loop through all markers visible to camera
    # If marker is visible, transform it to reference/base and store as PoseStamped object
    for i in range(len(message.markers)): 
      if message.markers[i].id == 1: # MARKER 1: ar_marker_1 is visible
        m1 = getWFPoseStamped('ar_marker_1')
        # note marker availability
        avail_ids.append(1) # add 1 to available marker ids
        pose_dict[1] = m1 # add m1 to dictionary of markers
        obj_ref_marker = 1 # list m1 as reference marker

        # store center point coordinates in xy frame - CHECK THIS!!!
        m1x = m1.pose.position.x 
        m1y = m1.pose.position.y
        z_dict[1] = m1.pose.position.z
        cpx = m1x - obj_width/2 # x axis to human operator's right (human facing markers 1(R) and 2(L))
        cpy = m1y + obj_length/2 # y axis to human operator's forward
      elif message.markers[i].id == 2: # MARKER 2: ar_marker_2 is visible
        m2 = getWFPoseStamped('ar_marker_2')
        # note marker availability
        avail_ids.append(2)
        pose_dict[2] = m2
        obj_ref_marker = 2

        # store center point coordinates in xy frame - CHECK THIS!!!
        m2x = m2.pose.position.x 
        m2y = m2.pose.position.y 
        z_dict[2] = m2.pose.position.z
        cpx = m2x + obj_width/2 # x axis to human operator's right
        cpy = m2y + obj_length/2 # y axis to human operator's forward
      elif message.markers[i].id == 3: # MARKER 3: ar_marker_3 is visible
        m3 = getWFPoseStamped('ar_marker_3')
        # note marker availability
        avail_ids.append(3)
        pose_dict[3] = m3
        obj_ref_marker = 3

        # store center point coordinates in xy frame - CHECK THIS!!!
        m3x = m3.pose.position.x 
        m3y = m3.pose.position.y 
        z_dict[3] = m3.pose.position.z
        cpx = m3x + obj_width/2 # x axis to human operator's right
        cpy = m3y - obj_length/2 # y axis to human operator's forward
      elif message.markers[i].id == 4: # MARKER 4: ar_marker_4 is visible
        m4 = getWFPoseStamped('ar_marker_4')
        # note marker availability
        avail_ids.append(4)
        pose_dict[4] = m4
        ojb_ref_marker = 4

        # store center point coordinates in xy frame - CHECK THIS!!!
        m4x = m4.pose.position.x 
        m4y = m4.pose.position.y 
        z_dict[4] = m4.pose.position.z
        cpx = m4x - obj_width/2 # x axis to human operator's right
        cpy = m4y - obj_length/2 # y axis to human operator's forward
      elif message.markers[i].id == 5: # HUMAN MARKER: ar_marker_5 is visible
        human_ar = getWFPoseStamped('ar_marker_5')
        # note marker availability
        hum_ref_marker = 5

    # print error messages
    if obj_ref_marker == -1: # looped through and could not find any object markers
      print('no object markers detected')
    if hum_ref_marker == -1: # looped through and could not find human marker
      print('human marker not detected')

    # object shape: distances between pairs of markers
    # calculated previously by sawyer_vision_calib.py
    obj_length = args[0]
    obj_width = args[1]
    print(obj_length)
    print(obj_width)

    # find missing markers and interpolate them
    missing_ids = list(set(allobj_ids) - set(avail_ids))
    print(missing_ids)



    # BELOW IS STILL INCOMPLETE: DEALING WITH MISSING MARKER PART
    # loop through the list of missing_ids and update it as missing markers are interpolated
    for j in range(length(missing_ids)): 
      if 1 in missing_ids: # MARKER 1 NOT VISIBLE- use 2 or 4 to reconstruct
        
        m1 = copy.deepcopy(pose_dict[obj_ref_marker]) # deepcopy PoseStamped from reference marker
        m1.pose.position.x = cpx - obj_width/2 # overwrite x coordinate
        m1.pose.position.y = cpy + obj_length/2 # overwrite y coordinate
        
      if 2 in missing_ids: # MARKER 2 NOT VISIBLE- use 1 or 3 to reconstruct

        m2 = copy.deepcopy(pose_dict[obj_ref_marker])
        m2.pose.position.x = cpx + obj_width/2
        m2.pose.position.y = cpy + obj_length/2

      if 3 in missing_ids: # MARKER 3 NOT VISIBLE - use 2 or 4 to reconstruct

        m3 = copy.deepcopy(pose_dict[obj_ref_marker])
        m3.pose.position.x = cpx + obj_width/2
        m3.pose.position.y = cpy - obj_length/2

      if 4 in missing_ids: # MARKER 4 NOT VISIBLE - use 1 or 3 to reconstruct

        m4 = copy.deepcopy(pose_dict[ojb_ref_marker])
        m4.pose.position.x = cpx - obj_width/2
        m4.pose.position.y = cpy - obj_length/2
   


   
    pub.publish(VisualData(obj_length,obj_width,m1,m2,m3,m4,human_ar))
    print('publishing to /tag_info')
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pass



# define the marker_locator method which contains the main functionality of the node
def marker_locator(length,width):
  # create a subscriber to subscribe to /ar_pose_marker to get marker locations
  # once marker locations found, run callback function
  r = rospy.Rate(20) #20 Hz

  while not rospy.is_shutdown():
    # add length and width from command line as arguments to callback
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback, (length,width)) 
    r.sleep()



# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Run this program as a new node in the ROS computation graph called /sawyer_vision.
  rospy.init_node('sawyer_vision', anonymous=True)

  # object dimensions calculated from sawyer_vision_calib.py
  # pass these in as command line arguments
  length = sys.argv[1]
  width = sys.argv[2]

  # create instances of tfBuffer and tfListener
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)

  # Check if the node has received a signal to shut down
  # If not, run the marker_locator method
  try:
    print ('Sawyer Vision is Running')
    marker_locator(length,width)
  except rospy.ROSInterruptException:
    pass
