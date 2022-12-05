#!/usr/bin/env python
import rospy
import tf2_ros

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from ar_track_alvar_msgs.msg import AlvarMarker # built-in message type for 1 AR tag
from ar_track_alvar_msgs.msg import AlvarMarkers # built-in message type for multiple AR tags

 

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform

import numpy as np

# class processor(object):
#      def __init__(self): 
        
#         # Create Node
#         rospy.init_node('ar2waypoint')

#         # Publisher to Publish the waypoint
#         pub = rospy.Publisher('/waypoint', Pose, queue_size=10)
        
#         # Define target Frames
#         target_frame = "ar_tag8"
#         effector_frame = "right_gripper_base"  

#         # TF Listener
#         tfBuffer = tf2_ros.Buffer()
#         tfListener = tf2_ros.TransformListener(tfBuffer)

#         # Create a timer object that will sleep long enough to result in
#         # a 10Hz publishing rate
#         r = rospy.Rate(3) # 3hz

#         while not rospy.is_shutdown():
#             try:

#                 # Create the waypoint object
#                 trans = tfBuffer.lookup_transform(target_frame, effector_frame, rospy.Time())
#                 print(type(trans))
#                 trans = trans.transform
#                 space = 0.2  # Distance between the tag and the effector
#                 waypoint = Pose()
#                 waypoint.position = Point(x = trans.x + space, y = trans.y, z = trans.z)
#                 waypoint.orientation.y = 1.0

#                 # Publish the waypoint for the moveit node to subscribe to
#                 self.pub.publish(waypoint)

#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 pass

#             # Use our rate object to sleep until it is time to publish again
#             r.sleep()

class Processor(object):
    def __init__(self): 
        
        # Create Node
        rospy.init_node('ar2waypoint')
        print("Node Started!")

        # Publisher to Publish the waypoint
        pub = rospy.Publisher('/waypoint', Pose, queue_size=10)
        # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        
        # Define target Frames
        target_frame = "ar_marker_11"
        effector_frame = "head_camera"  

        # TF Listener
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        # Create a timer object that will sleep long enough to result in
        # a 10Hz publishing rate
        r = rospy.Rate(2) # 3hz

        x_start = -0.3662
        x_end = 0.6842
        num_points = 50

        # Initialize position
        p1 = Pose()
        p1.position = Point(x = 0.6, y= x_start, z = self.f(x_start))
        p1.orientation.y = 1.0
        pub.publish(p1)

        print("Pringing A: ")
        points = []

        step_size = float(abs(x_start - x_end))/(num_points-1)
        print(step_size)
        ranges = np.arange(x_start, x_end, step_size)
        print(ranges)
        for i in ranges:
            a = [0.6, i, self.f(i)]
            print("Pringing A: ", a)
            points.append(a)
        print(points)

        # point = [[0.6, -0.211, 0.2], [0.6, -0.255, 0.3], [0.6, 0.159, 0.4], [0.6, 0.5733, 0.3], [0.691, 0.6, 0.2]]
        # quart = [0.0, 1.0, 0.0, 0.0]

        # while(True):
        #     pub.publish(p1)
        #     print("Published!")
        for i in points[:3]:
            p1 = Pose()
            p1.position = Point(x = i[0], y = i[1], z = i[2])
            p1.orientation.y = 1.0
            pub.publish(p1)
            print(p1)
            print("Published!")
            r.sleep()
        # print("Going Into Loop")

    # test function
    def f(self, x):
        return -1.45*(x-0.159)**2+0.4

        # while not rospy.is_shutdown():
        #     try:
        #         # Create the waypoint object
        #         print("Trying Transform!")
        #         trans = tfBuffer.lookup_transform(target_frame, effector_frame, rospy.Time())
        #         # print(type(trans))
        #         trans = trans.transform.translation
        #         space = 0.2  # Distance between the tag and the effector
        #         waypoint = Pose()
        #         waypoint.position = Point(x = trans.x - space, y = trans.y, z = trans.z)
        #         waypoint.orientation.y = 1.0

        #         # Publish the waypoint for the moveit node to subscribe to
        #         pub.publish(waypoint)
        #         print("published!")

        #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #         pass

        #     # Use our rate object to sleep until it is time to publish again
        #     r.sleep()

    # def callback(self):


    # while not rospy.is_shutdown():
    #     try:

    #         # Create the waypoint object
    #         trans = tfBuffer.lookup_transform(target_frame, effector_frame, rospy.Time())
    #         print(type(trans))
    #         trans = trans.transform
    #         space = 0.2  # Distance between the tag and the effector
    #         waypoint = Pose()
    #         waypoint.position = Point(x = trans.x + space, y = trans.y, z = trans.z)
    #         waypoint.orientation.y = 1.0

    #         # Publish the waypoint for the moveit node to subscribe to
    #         self.pub.publish(waypoint)

    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         pass

    #     # Use our rate object to sleep until it is time to publish again
    #     r.sleep()


if __name__ == '__main__':
    # rospy.init_node('ar2waypoint')
    p = Processor()