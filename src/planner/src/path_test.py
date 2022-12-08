#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
# import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

from path_planner import PathPlanner
import matplotlib.pyplot as plt
from vision.msg import VisualData

import pickle

import os


try:
    from controller import Controller
except ImportError:
    pass
    
# def main():
#     """
#     Main Script
#     """
#     # Make sure that you've looked at and understand path_planner.py before starting
#     planner = PathPlanner("right_arm")

#     # Create Waypoints
#     point = [[0.6, -0.211, 0.2], [0.6, -0.255, 0.3], [0.6, 0.159, 0.4], [0.6, 0.5733, 0.3], [0.691, 0.6, 0.2]]
#     quart = [0.0, 1.0, 0.0, 0.0]
#     waypoints = []

#     for i in point:
#         p1 = Pose()
#         p1.position = Point(x = i[0], y = i[1], z = i[2])
#         p1.orientation.y = 1.0
#         waypoints.append(p1)

#     print(waypoints)

#     while not rospy.is_shutdown():

#         go_flag = False
#         while not go_flag and not rospy.is_shutdown():
#             try:
#                 plan = planner.plan_to_pose(waypoints)#[orien_const])

#                 user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
#                 if user_input == 'y':
#                     input("Press <Enter> to move the right arm to goal pose 1: ")
#                     if not planner.execute_plan(plan): 
#                         raise Exception("Execution failed")
#                     go_flag = True
#                 else:
#                     print("Generating New Path")
#                     pass
#             except Exception as e:
#                 print(e)
#                 traceback.print_exc()
#             # else:
#             #     break

# class PathExecutor(object):
#     def __init__(self): # group of joints name

#         # Initialize node
#         rospy.init_node('moveit_node')

#         # Subscribe to the waypoint topic from ar_to_waypoint
#         rospy.Subscriber("/waypoint", Pose, self.callback)

#         # Planner
#         self.planner = PathPlanner("right_arm")
#         pub = rospy.Publisher('/waypoint', String, queue_size=10)
#         mssg = "I'm Online"
#         while(True):
#             pub.publi
#         # running the path
#         while not go_flag and not rospy.is_shutdown():
#             try:
#                 # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
#                 # if user_input == 'y':
#                 #     print("User Pressed Y")
#                 #     input("Press <Enter> to move the right arm to goal pose 1: ")
#                 if not self.planner.execute_plan(plan): 
#                     raise Exception("Execution failed")
#                 go_flag = True
#             except Exception as e:
#                 print(e)
#                 traceback.print_exc()

#     def run(self):
#         rospy.spin()
#         # convert the transformation to a waypoint path for execution and then publish the waypoint
#         go_flag = False
        
#         # Planning the path now
#         plan = self.planner.plan_to_pose([waypoint])
        
#         # running the path
#         while not go_flag and not rospy.is_shutdown():
#             try:
#                 # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
#                 # if user_input == 'y':
#                 #     print("User Pressed Y")
#                 #     input("Press <Enter> to move the right arm to goal pose 1: ")
#                 if not self.planner.execute_plan(plan): 
#                     raise Exception("Execution failed")
#                 go_flag = True
#             except Exception as e:
#                 print(e)
#                 traceback.print_exc()

#     def run(self):
#         rospy.spin()


# class PathExecutor(object):
#     def __init__(self): # group of joints name

#         # Initialize node
#         rospy.init_node('moveit_node')

#         # Subscribe to the waypoint topic from ar_to_waypoint
#         rospy.Subscriber("/waypoint", Pose, self.callback)

#         # Planner
#         self.planner = PathPlanner("right_arm")
#         # Controller
#         Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
#         Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
#         Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
#         Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

#         self.controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

#         # pub = rospy.Publisher('/waypoint', String, queue_size=10)
#         # mssg = "I'm Online"
#         # while(True):
#         #     pub.publish(mssg)
#         self.counter = 0
        
#         # Make 2 subplots, one for the velocities magnitude and
#         # one for the number of points for a given trajectory
#         x = np.array([0,1 ,2])
#         y = np.array([0,1 ,2])
#         fig, axs = plt.subplots(2, 4)
#         axs = axs.flat
#         for i, ax in enumerate(axs[:-1]):
#             ax.set_title('theta' + str(i))
#             ax.set(xlabel = 'vel_no.', ylabel = 'vel_magnitude')

#         axs[-1].set_title('NumPoints')
#         axs[-1].set(xlabel = 'point_no.', ylabel = 'num_poses')


#         xs = np.empty((7, 0))
#         ys = np.empty((7, 0))
#         x_c = []
#         y_c = []

#         # Creating Test Data
#         x_start = -0.3662
#         x_end = 0.6842
#         num_points = 50
#         r = rospy.Rate(2) # 3hz

#         def f(x):
#             return -1.45*(x-0.159)**2+0.4

#         print("Printing A: ")
#         points = []

#         step_size = float(abs(x_start - x_end))/(num_points-1)
#         print(step_size)
#         ranges = np.arange(x_start, x_end, step_size)
#         print(ranges)
#         for i in ranges:
#             a = [0.6, i, f(i)]
#             print("Printing A: ", a)
#             points.append(a)

#         print(points)

#         # Creating Test Data
#         for z, i in enumerate(points):
#             p1 = Pose()
#             p1.position = Point(x = i[0], y = i[1], z = i[2])
#             p1.orientation.y = 1.0

#             plan = self.planner.plan_to_pose([p1])
#             trajectory = plan.joint_trajectory.points

#             # Record the trajectory Length
#             y_c.append(len(trajectory))

#             # clear the plot lists
#             xs = np.empty((7, 0))
            

#             for i in trajectory:
#                 temp = np.array([i.velocities]).T
#                 xs = np.hstack((xs, temp))
                
#             y = np.arange(len(xs[0]))
#             for j, x in enumerate(xs):
#                 # axs[i].scatter(ys[i], x, s= 10)
#                 axs[j].plot(y, x, label=z)
#                 axs[j].legend(loc='upper center')
                

#         axs[-1].scatter(np.arange(len(y_c)), y_c, s=10)
#         plt.show()

#         # print(trajectory)

#     # Callback function whenever a waypoint is received
#     def callback(self, waypoint):
#         print("Waypoint #", self.counter, " received!")
#         self.counter += 1

#         # convert the transformation to a waypoint path for execution and then publish the waypoint
#         go_flag = False
        
#         # Planning the path now
#         plan = self.planner.plan_to_pose([waypoint])
        
#         # running the path
#         while not go_flag and not rospy.is_shutdown():
#             try:
#                 # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
#                 # if user_input == 'y':
#                 #     print("User Pressed Y")
#                 #     input("Press <Enter> to move the right arm to goal pose 1: ")
#                 if not self.planner.execute_plan(plan): 
#                     raise Exception("Execution failed")
#                 go_flag = True
#             except Exception as e:
#                 print(e)
#                 traceback.print_exc()

#     def run(self):
#         rospy.spin()

#     def update_target(self, ):


# if __name__ == '__main__':
#     # rospy.init_node('moveit_node')
#     pe = PathExecutor()
#     pe.run()
#     # main()


class PathExecutor(object):
    def __init__(self): # group of joints name

        # Initialize node
        rospy.init_node('moveit_node')
        print("Node Initialized!")
        self.cwd = "/home/cc/ee106a/fa22/class/ee106a-ahh/ros_workspaces/finalproject2/src/planner/src/"
        os.chdir(self.cwd)
        print(os.getcwd())

        # Planner
        self.planner = PathPlanner("right_arm")
        self.counter = 0

        # FINAL CODE STARTS HERE
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        controller = Controller(Kp, Kd, Ki, Kw, Limb("right"))
        
        # Initialize signal
        p1 = Pose()
        p1.position = Point(x = 0.691, y= 0.159, z = 0.386)
        p1.orientation.y = 1.0

        p2 = Pose()
        p2.position = Point(x = 0.691, y= 0.43, z = 0.386)
        p2.orientation.y = 1.0

        p3 = Pose()
        p3.position = Point(x = 0.691, y= 0.43, z = -0.12)
        p3.orientation.y = 1.0

        p4 = Pose()
        p4.position = Point(x = 0.691, y= 0.23, z = -0.12)
        p4.orientation.y = 1.0

        plan = self.planner.plan_to_pose([p1, p2, p3, p4])
        if not controller.execute_plan(plan): 
            raise Exception("Execution failed")

        # Send start signal to controller_executor
        with open('start.pickle', 'wb') as handle:
            a = 1
            pickle.dump(a, handle, protocol=pickle.HIGHEST_PROTOCOL)
            # print("Dumped!")


        r = rospy.Rate(35) # 3hz
        z_start = -0.15
        z_end = 0.05
        num_points = 50

        step_size = float(abs(z_start - z_end))/(num_points-1)
        ranges = np.arange(z_start, z_end, step_size)
        print(step_size)
        # ranges = np.arange(x_start, x_end, step_size)
        points = []
        print(ranges)
        for i in ranges:
            a = [0.691, 0.2, i]
            print("Printing A: ", a)
            points.append(a)

        for i in points:
            p1 = Pose()
            p1.position = Point(x = i[0], y = i[1], z = i[2])
            f = [0.015, 0.719, -0.693, -0.049]
            p1.orientation.y = 1
            # p1.orientation.x = f[0]
            # p1.orientation.y = f[1]
            # p1.orientation.z = f[2]
            # p1.orientation.w = f[3]

            plan = self.planner.plan_to_pose([p1])
            trajectory = plan.joint_trajectory.points
            print("Target Velocities: ", trajectory[-2].velocities)
            print("Target Positions: ",trajectory[-1].positions)

            target_pos = trajectory[-1].positions 
            target_vel = trajectory[-2].velocities 

            # Write the targets to a pickle value that will be read by the executioner
            a = (target_pos, target_vel)
            with open('comm.pickle', 'wb') as handle:
                pickle.dump(a, handle, protocol=pickle.HIGHEST_PROTOCOL)
                # print("Dumped!")

            r.sleep()

        # FEEDBACK CONTROL
        # Subscribe to the waypoint topic from ar_to_waypoint
        self.repeat()
        # # # testing
        # x_start = -0.3662
        # x_end = 0.6842
        # num_points = 50

        # # # Initialize position
        # def f(x):
        #     return -1.45*(x-0.159)**2+0.4

        # Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        # Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        # Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        # Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        # controller = Controller(Kp, Kd, Ki, Kw, Limb("right"))

        # step_size = float(abs(x_start - x_end))/(num_points-1)
        # ranges = np.arange(x_start, x_end, step_size)
        # print(step_size)
        # ranges = np.arange(x_start, x_end, step_size)
        # points = []
        # print(ranges)
        # for i in ranges:
        #     a = [0.639, i, f(i)]
        #     # print("Printing A: ", a)
        #     points.append(a)

        # for p in points:
        #     try:
        #         p1 = Pose()
        #         p1.position = Point(x = p[0], y= p[1], z = p[2])
        #         p1.orientation.y = 1.0
        #         plan = self.planner.plan_to_pose([p1])
        #         # plan = self.planner.plan_to_pose(p1, [])
        #         print(plan)
        #         if not controller.execute_plan(plan): 
        #             raise Exception("Execution failed")
        #     except Exception as e:
        #         print(e)
        #         traceback.print_exc()
        #     # break
    
    def repeat(self):
        rospy.Subscriber("/waypoint", Pose, self.callback, queue_size=30)
        rospy.spin()

    # Callback function whenever a waypoint is received (how long are you going to update this value?)
    def callback(self, waypoint):
        t = (rospy.Time.now()).to_sec()
        print("Received Waypoint!, time: ", t)
        print("Waypoint", waypoint)
        try:
            plan = self.planner.plan_to_pose([waypoint])
            trajectory = plan.joint_trajectory.points
            
            # get the second last point in the velocity vector
            # get the target_position
            # assumption is that, if the time difference is small enough, the 2nd last point is representative of the velicoties
            target_pos = trajectory[-1].positions 
            target_vel = trajectory[-2].velocities 
            
            # update the target position of the controller
            a = (target_pos, target_vel)
            with open('comm.pickle', 'wb') as handle:
                pickle.dump(a, handle, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as e:
            print("couldn't Write File")
    
    # def run(self):
        


if __name__ == '__main__':
    # rospy.init_node('moveit_node')
    PathExecutor()
    # pe.run()
    rospy.spin()
    # main()