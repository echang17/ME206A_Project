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

# import intera_interface
# from intera_interface import Limb

from geometry_msgs.msg import PoseStamped # pose data of joints/markers
from geometry_msgs.msg import TransformStamped # transform data between joints/markers
from vision.msg import VisualData
from ar_track_alvar_msgs.msg import AlvarMarker # built-in message type for 1 AR tag
from ar_track_alvar_msgs.msg import AlvarMarkers # built-in message type for multiple AR tags

import tf.transformations as tr
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

# rot_matrix = transform.transform.rotation

pub = rospy.Publisher('/tag_info', VisualData, queue_size=10) # left queue_size=10 from lab2

def transform_wf(pose_obj, rotation_w, translation_w):
  # function to convert PoseStamped objects read from head_camera frame to world frame
  pose = pose_obj.pose
  p_trans = pose.position
  p_rot = pose.orientation
  rotation_p = R.from_quat([p_rot.x, p_rot.y, p_rot.z, p_rot.w]).as_dcm()
  translation_p = [p_trans.x, p_trans.y, p_trans.z]
  print(rotation_w.as_dcm(), rotation_p)
  r_new = np.dot(np.asarray(rotation_w.as_dcm()), np.asarray(rotation_p))#.as_matrix()
  print(r_new)
  o_new = R.from_dcm(r_new).as_quat()# convert back to quaternion
  print(np.asarray(o_new))
  # translation_w = 
  ptvector = np.array([p_trans.x,p_trans.y,p_trans.z])
  t_new = translation_w + ptvector
  print("New rotation", r_new)
  print("old rotation", rotation_p)
  pose_new = PoseStamped()
  pose_new.pose.position.x = t_new[0]
  pose_new.pose.position.y = t_new[1]
  pose_new.pose.position.z = t_new[2]
  pose_new.pose.orientation.x = o_new[0]
  pose_new.pose.orientation.y = o_new[1]
  pose_new.pose.orientation.z = o_new[2]
  pose_new.pose.orientation.w = o_new[3]

  pose_new.header = copy.deepcopy(pose_obj.header)
  return pose_new



# define a callback method which is called whenever this node receives a 
# message on its subscribed topic. received message is passes as argument to callback()
def callback(message):
  # while not rospy.is_shutdown():
  try:

    print('try')
    
    # transform = tfListener.lookupTransform(base_frame, cam_frame, rospy.Time(1.0)).transform
    # time.sleep(2)

    transform = tfBuffer.lookup_transform('reference/base', 'ar_marker_1', rospy.Time()).transform  
    translation = transform.translation
    translation_w = [translation.x, translation.y, translation.z]
    rot = transform.rotation
    m1 = PoseStamped()
    m1.pose.orientation = rot#.x = tr.quaternion_from_matrix(rot).x
    m1.pose.position = translation#.x = translation.x
    # rotation_w = tr.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])




    # read and save AR tag information from /ar_pose_marker topic
    # print('number of markers detected:')
    # print(len(message.markers))
    allobj_ids = [1,2,3,4]
    avail_ids = []
    dict = {}
    ref_marker = -1
    
    #object shape: distances between pairs of markers
    # length: need both 1&4 opr 2&3 KNOWN CONSTANT: 0.23282 for 8x11 rectangle object
    # width: need both 1&2 or 3&4 KNOWN CONSTANT: 0.15835 for 8x11 rectangle object
    # obj_length = 0.005659
    # obj_width = 0.01973
    


    # for i in range(len(message.markers)): # loop through all markers detected
    #   if message.markers[i].id == 1: # ar_marker_1
    #     m1 = PoseStamped()
    #     m1.header = message.markers[i].header # copy over header for ar_marker_1
    #     m1.pose = message.markers[i].pose.pose # copy over pose for ar_marker_1

    #     avail_ids.append(1) # add 1 to available marker ids
    #     dict[1] = m1 # add m1 to dictionary of markers
    #     ref_marker = 1 # list m1 as reference marker

    #     m1x = m1.pose.position.x # x axis to human operator's right
    #     m1y = m1.pose.position.y # y axis to human operator's forward
    #     m1z = m1.pose.position.z # z axis points up

    #     # cpx = m1x + obj_width/2
    #     # cpy = m1y - obj_length/2
    #   elif message.markers[i].id == 2: # ar_marker_2
    #     m2 = PoseStamped()
    #     m2.header = message.markers[i].header
    #     m2.pose = message.markers[i].pose.pose

    #     avail_ids.append(2)
    #     dict[2] = m2
    #     ref_marker = 2

    #     m2x = m2.pose.position.x # x axis to human operator's right
    #     m2y = m2.pose.position.y # y axis to human operator's forward
    #     m2z = m2.pose.position.z # z axis points up

    #     # cpx = m2x - obj_width/2
    #     # cpy = m2y - obj_length/2
    #   elif message.markers[i].id == 3: # ar_marker_3
    #     m3 = PoseStamped()
    #     m3.header = message.markers[i].header
    #     m3.pose = message.markers[i].pose.pose

    #     avail_ids.append(3)
    #     dict[3] = m3
    #     ref_marker = 3

    #     m3x = m3.pose.position.x # x axis to human operator's right
    #     m3y = m3.pose.position.y # y axis to human operator's forward
    #     m3z = m3.pose.position.z # z axis points up

    #     # cpx = m3x - obj_width/2
    #     # cpy = m3y + obj_length/2
    #   elif message.markers[i].id == 4: # ar_marker_4
    #     m4 = PoseStamped()
    #     m4.header = message.markers[i].header
    #     m4.pose = message.markers[i].pose.pose

    #     avail_ids.append(4)
    #     dict[4] = m4
    #     ref_marker = 4

    #     m4x = m4.pose.position.x # x axis to human operator's right
    #     m4y = m4.pose.position.y # y axis to human operator's forward
    #     m4z = m4.pose.position.z # z axis points up

    #     # cpx = m4x + obj_width/2
    #     # cpy = m4y + obj_length/2
    #   elif message.markers[i].id == 5: # marker id 5 = ar_marker_5
    #     human_ar = PoseStamped()
    #     human_ar.header = message.markers[i].header
    #     human_ar.pose = message.markers[i].pose.pose
    #     # avail_ids.append(5)


    # print(m1.header.frame_id) # m1.header.frame_id
    # transform1 = tfBuffer.lookup_transform('head_camera', 'ar_marker_1', rospy.Time()).transform  
    # translation1 = transform1.translation
    # translation_w1 = [translation1.x, translation1.y, translation1.z]
    # rot1 = transform1.rotation
    # print(rot1.x)
    # rotation_w1 = tr.quaternion_matrix([rot1.x, rot1.y, rot1.z, rot1.w])
    # print(np.asarray(rotation_w).shape, np.asarray(rotation_w1).shape)

    # rotation_total_1 = np.dot(np.asarray(rotation_w), np.asarray(rotation_w1))
    # translation_total_1 = translation_w1 + translation_w
    # m1 = PoseStamped()
    # # pm1.header = 'artag1'
    # # pm1.header.frame_id = 'artag1'
    # m1.pose.orientation.x = tr.quaternion_from_matrix(rotation_total_1)[0]
    # m1.pose.orientation.y = tr.quaternion_from_matrix(rotation_total_1)[1]
    # m1.pose.orientation.z = tr.quaternion_from_matrix(rotation_total_1)[2]
    # m1.pose.orientation.w = tr.quaternion_from_matrix(rotation_total_1)[3]
    # m1.pose.position.x = translation_total_1[0]
    # m1.pose.position.y = translation_total_1[1]
    # m1.pose.position.z = translation_total_1[2]

    # transform2 = tfBuffer.lookup_transform('head_camera', 'ar_marker_2', rospy.Time()).transform  
    # translation2 = transform2.translation
    # translation_w2 = [translation2.x, translation2.y, translation2.z]
    # rot2 = transform2.rotation
    # rotation_w2 = tr.quaternion_matrix([rot2.x, rot2.y, rot2.z, rot2.w])
    # rotation_total_2 = np.dot(np.asarray(rotation_w), np.asarray(rotation_w2))
    # translation_total_2 = translation_w2 + translation_w
    # m2 = PoseStamped()
    # m2.pose.orientation.x = tr.quaternion_from_matrix(rotation_total_2)[0]
    # m2.pose.orientation.y = tr.quaternion_from_matrix(rotation_total_2)[1]
    # m2.pose.orientation.z = tr.quaternion_from_matrix(rotation_total_2)[2]
    # m2.pose.orientation.w = tr.quaternion_from_matrix(rotation_total_2)[3]
    # m2.pose.position.x = translation_total_2[0]
    # m2.pose.position.y = translation_total_2[1]
    # m2.pose.position.z = translation_total_2[2]
    # print(m2)
    transform2 = tfBuffer.lookup_transform('reference/base', 'ar_marker_2', rospy.Time()).transform  
    translation = transform2.translation
    translation_w = [translation.x, translation.y, translation.z]
    rot = transform2.rotation
    m2 = PoseStamped()
    m2.pose.orientation = rot#.x = tr.quaternion_from_matrix(rot).x
    m2.pose.position = translation#.x = translation.x

    print('test2')

    transform3 = tfBuffer.lookup_transform('reference/base', 'ar_marker_3', rospy.Time()).transform  
    translation = transform3.translation
    translation_w = [translation.x, translation.y, translation.z]
    rot = transform3.rotation
    m3 = PoseStamped()
    m3.pose.orientation = rot#.x = tr.quaternion_from_matrix(rot).x
    m3.pose.position = translation#.x = translation.x

    print('test3')

    transform4 = tfBuffer.lookup_transform('reference/base', 'ar_marker_4', rospy.Time()).transform  
    translation = transform4.translation
    translation_w = [translation.x, translation.y, translation.z]
    rot = transform4.rotation
    m4 = PoseStamped()
    m4.pose.orientation = rot#.x = tr.quaternion_from_matrix(rot).x
    m4.pose.position = translation#.x = translation.x


    print('test4')


    transform5 = tfBuffer.lookup_transform('reference/base', 'ar_marker_5', rospy.Time()).transform  
    translation = transform5.translation
    translation_w = [translation.x, translation.y, translation.z]
    rot = transform5.rotation
    m5 = PoseStamped()
    m5.pose.orientation = rot#.x = tr.quaternion_from_matrix(rot).x
    m5.pose.position = translation#.x = translation.x
    human_ar = m5


    # DO NOT UPDATE THESE PARAMETERS, CALCULATE THEM BEFORE EVERYTHING
    obj_length = m2.pose.position.y - m3.pose.position.y 
    obj_width = m2.pose.position.x - m1.pose.position.x
    print(obj_length)
    print(obj_width)

    # # find missing markers and interpolate them
    missing_ids = list(set(allobj_ids) - set(avail_ids))
    # # print(avail_ids) # object ids only- exclude human ar tag
    # print(missing_ids)
    # if ref_marker == -1:
    #   print('no object markers detected')

    # if 1 in missing_ids:
    #   m1 = copy.deepcopy(dict[ref_marker]) # deepcopy PoseStamped from reference marker
    #   m1.pose.position.x = cpx - obj_width/2 # overwrite x coordinate
    #   m1.pose.position.y = cpy + obj_length/2 # overwrite y coordinate
    # if 2 in missing_ids:
    #   m2 = copy.deepcopy(dict[ref_marker])
    #   m2.pose.position.x = cpx + obj_width/2
    #   m2.p print(obj_length)
    #print(obj_width)ose.position.y = cpy + obj_length/2
    # if 3 in missing_ids:
    #   m3 = copy.deepcopy(dict[ref_marker])
    #   m3.pose.position.x = cpx + obj_width/2
    #   m3.pose.position.y = cpy - obj_length/2
    # if 4 in missing_ids:
    #   m4 = copy.deepcopy(dict[ref_marker])
    #   m4.pose.position.x = cpx - obj_width/2
    #   m4.pose.position.y = cpy - obj_length/2

    # transform tags to world frame 
    # pm1 = transform_wf(m1, rotation_w, translation_w) # m1_wf
    # m2_wf = transform_wf(m2, rotation_w, translation_w)
    # m3_wf = transform_wf(m3, rotation_w, translation_w)
    # m4_wf = transform_wf(m4, rotation_w, translation_w)
    # # human_ar_wf = transform_wf(human_ar, rotation_w, translation_w)
    # pm1 = transform_wf(m1, rotation_w, translation_w) # m1_wf
    # pm2 = transform_wf(m2, rotation_w, translation_w)
    # pm3 = transform_wf(m3, rotation_w, translation_w)
    # pm4 = transform_wf(m4, rotation_w, translation_w)
    # human_ar_wf = transform_wf(human_ar, rotation_w, translation_w)


    # print("New pose", m1_wf)
    # print("old pose", m1)
    # print('lalala')
   
    pub.publish(VisualData(obj_length,obj_width,m1,m2,m3,m4,human_ar))
    print('publishing to /tag_info')
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pass

# define the method which contains the main functionality of the node
def marker_locator():
  # create a subscriber to subscribe to /ar_pose_marker to get marker locations
  
  r = rospy.Rate(20)
  while not rospy.is_shutdown():
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback)
    # rospy.Subscriber('')
    r.sleep()
  # rospy.spin()



# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Run this program as a new node in the ROS computation graph called /sawyer_vision.
  rospy.init_node('sawyer_vision', anonymous=True)

  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  # base_frame = reference/base
  # cam_frame = reference/head_camera
  # base_frame = 'ar_marker_4'
  # cam_frame = 'head_camera'

  # tfBuffer = tf2_ros.Buffer()
  # tfListener = tf2_ros.TransformListener(tfBuffer)
  # # transform = tfListener.lookupTransform(base_frame, cam_frame, rospy.Time(1.0)).transform
  # time.sleep(2)
  # transform = tfBuffer.lookup_transform('reference/base', 'reference/head_camera', rospy.Time(1.0)).transform  
  # time.sleep(2)
  # tr1 = tfBuffer.lookup_transform('reference/base', 'reference/head_camera', rospy.Time()).transform
  # tr2 = tfBuffer.lookup_transform('head_camera', m1.header.frame_id, rospy.Time()).transform
  # # transform = tfBuffer.lookup_transform(sys.argv[1], sys.argv[2], rospy.Time(1.0)).transform  
  # translation = transform.translation
  # translation_w = [translation.x, translation.y, translation.z]
  # rot = transform.rotation
  # rotation_w = R.from_quat([rot.x, rot.y, rot.z, rot.w])


  # Check if the node has received a signal to shut down
  # If not, run the talker method
  try:
    print ('Sawyer Vision is Running')
    marker_locator()
  except rospy.ROSInterruptException:
    pass
