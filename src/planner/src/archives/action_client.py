#!/usr/bin/env python
import roslib; roslib.load_manifest('planning')

import rospy
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveGroupFeedback, MoveGroupResult, JointConstraint, Constraints
import sys

def main():
    #Initialize the node
    rospy.init_node('moveit_client')
    
    # Create the SimpleActionClient, passing the type of the action
    # (MoveGroupAction) to the constructor.
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)

    # Wait until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveGroupGoal()
    
    #----------------Construct the goal message (start)----------------
    joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    
    #Set parameters for the planner    
    goal.request.group_name = 'right_arm'
    goal.request.num_planning_attempts = 1
    goal.request.allowed_planning_time = 5.0
    
    #Define the workspace in which the planner will search for solutions
    goal.request.workspace_parameters.min_corner.x = -1
    goal.request.workspace_parameters.min_corner.y = -1
    goal.request.workspace_parameters.min_corner.z = -1
    goal.request.workspace_parameters.max_corner.x = 1
    goal.request.workspace_parameters.max_corner.y = 1
    goal.request.workspace_parameters.max_corner.z = 1
    
    goal.request.start_state.joint_state.header.frame_id = "base"

    
    #Set the start state for the trajectory
    goal.request.start_state.joint_state.name = joint_names
    goal.request.start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal.request.start_state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    #Tell MoveIt whether to execute the trajectory after planning it
    goal.planning_options.plan_only = True
    
    #Set the goal position of the robot
    #Note that the goal is specified with a collection of individual
    #joint constraints, rather than a vector of joint angles
    arm_joint_names = joint_names
    target_joint_angles = [1.0, 1.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    tolerance = 0.0001
    consts = []

    if (len(sys.argv) == 9):
        for i in range(0, 4):
            goal.request.start_state.joint_state.position[i] = float(sys.argv[i+1])
            print(float(sys.argv[i+1]))
        for i in range(0, 4):
            target_joint_angles[i] = float(sys.argv[i+5])
            print(float(sys.argv[i+5]))

    for i in range(len(arm_joint_names)):
        const = JointConstraint()
        const.joint_name = arm_joint_names[i]
        const.position = target_joint_angles[i]
        const.tolerance_above = tolerance
        const.tolerance_below = tolerance
        const.weight = 1.0
        consts.append(const)
        
    goal.request.goal_constraints.append(Constraints(name='', joint_constraints=consts))
    #---------------Construct the goal message (end)-----------------

    # Send the goal to the action server.
    client.send_goal(goal)

    # Wait for the server to finish performing the action.
    client.wait_for_result()

    # Print out the result of executing the action
    print(client.get_result())
    

if __name__ == '__main__':
    print("args: ", sys.argv)
    main()



# the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
# args:  ['action_client.py', '0', '0', '0', '0', '1', '1', '0.3', '0.3']
# 0.0
# 0.0
# 0.0
# 0.0
# 1.0
# 1.0
# 0.3
# 0.3
# error_code: 
#   val: 1
# trajectory_start: 
#   joint_state: 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 0
#         nsecs:         0
#       frame_id: "base"
#     name: 
#       - right_j0
#       - head_pan
#       - right_j1
#       - right_j2
#       - right_j3
#       - right_j4
#       - right_j5
#       - right_j6
#       - right_gripper_l_finger_joint
#       - right_gripper_r_finger_joint
#     position: [0.0, 0.1231337890625, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     effort: []
#   multi_dof_joint_state: 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 0
#         nsecs:         0
#       frame_id: "base"
#     joint_names: []
#     transforms: []
#     twist: []
#     wrench: []
#   attached_collision_objects: []
#   is_diff: False
# planned_trajectory: 
#   joint_trajectory: 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 0
#         nsecs:         0
#       frame_id: "base"
#     joint_names: 
#       - right_j0
#       - right_j1
#       - right_j2
#       - right_j3
#       - right_j4
#       - right_j5
#       - right_j6
#     points: 
#       - 
#         positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         effort: []
#         time_from_start: 
#           secs: 0
#           nsecs:         0
#       - 
#         positions: [0.16667054328297573, 0.1666739757733342, 0.049994426419772205, 0.04998557435871723, -1.2901875407745443e-05, 1.5050298239414891e-05, 1.5387247727873425e-05]
#         velocities: [0.6524817153578777, 0.6524951528685077, 0.19571814230738233, 0.19568348826553528, -5.050825197759604e-05, 5.891889603568675e-05, 6.0237985635371523e-05]
#         accelerations: [0.19938349824542348, 0.1993876044415268, 0.05980699070155242, 0.05979640121846157, -1.5434167322268767e-05, 1.8004267901837384e-05, 1.8407351532679158e-05]
#         effort: []
#         time_from_start: 
#           secs: 0
#           nsecs: 265831823
#       - 
#         positions: [0.33334108656595146, 0.3333479515466684, 0.09998885283954441, 0.09997114871743445, -2.5803750815490885e-05, 3.0100596478829782e-05, 3.077449545574685e-05]
#         velocities: [0.6779860372415536, 0.678, 0.20336840802731088, 0.20333239942208364, -5.248252755635592e-05, 6.122192837231059e-05, 6.259257878197962e-05]
#         accelerations: [4.516189208323211e-16, 0.0, 1.1290473020808026e-16, 0.0, 0.0, 5.512926279691419e-20, 0.0]
#         effort: []
#         time_from_start: 
#           secs: 0
#           nsecs: 511663645
#       - 
#         positions: [0.5000116298489272, 0.5000219273200026, 0.14998327925931662, 0.14995672307615168, -3.870562622323633e-05, 4.5150894718244676e-05, 4.6161743183620276e-05]
#         velocities: [0.6779860372415535, 0.678, 0.20336840802731088, 0.20333239942208364, -5.248252755635592e-05, 6.122192837231059e-05, 6.259257878197962e-05]
#         accelerations: [-9.032378416646421e-16, 0.0, -2.2580946041616053e-16, 0.0, 0.0, -1.1025852559382838e-19, 0.0]
#         effort: []
#         time_from_start: 
#           secs: 0
#           nsecs: 757495468
#       - 
#         positions: [0.6666821731319029, 0.6666959030933368, 0.19997770567908882, 0.1999422974348689, -5.160750163098177e-05, 6.0201192957659564e-05, 6.15489909114937e-05]
#         velocities: [0.6779860372415536, 0.678, 0.2033684080273109, 0.20333239942208364, -5.248252755635592e-05, 6.122192837231059e-05, 6.259257878197963e-05]
#         accelerations: [1.8064756833292842e-15, 0.0, 4.516189208323211e-16, 0.0, 0.0, 1.1025852559382838e-19, 1.1025852559382838e-19]
#         effort: []
#         time_from_start: 
#           secs: 1
#           nsecs:   3327291
#       - 
#         positions: [0.8333527164148787, 0.833369878866671, 0.24997213209886104, 0.24992787179358614, -6.450937703872721e-05, 7.525149119707446e-05, 7.693623863936713e-05]
#         velocities: [0.5672470259469011, 0.5672587080948625, 0.1701511805381729, 0.17012105340707923, -4.391028138520866e-05, 5.1222230081936744e-05, 5.2369004979633716e-05]
#         accelerations: [-0.7250512660603693, -0.7250661981019343, -0.21748607436932954, -0.21744756607656948, 5.6125821123992594e-05, -6.547190390174068e-05, -6.69377037269113e-05]
#         effort: []
#         time_from_start: 
#           secs: 1
#           nsecs: 249159113
#       - 
#         positions: [1.0000232596978544, 1.0000438546400052, 0.29996655851863324, 0.29991344615230336, -7.741125244647266e-05, 9.030178943648935e-05, 9.232348636724055e-05]
#         velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         accelerations: [-2.5007366429221234, -2.500788144250714, -0.7501199169761661, -0.7499871000247751, 0.00019358065293989126, -0.00022581574135935214, -0.00023087135536291054]
#         effort: []
#         time_from_start: 
#           secs: 1
#           nsecs: 614257946
#   multi_dof_joint_trajectory: 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 0
#         nsecs:         0
#       frame_id: ''
#     joint_names: []
#     points: []
# executed_trajectory: 
#   joint_trajectory: 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 0
#         nsecs:         0
#       frame_id: ''
#     joint_names: []
#     points: []
#   multi_dof_joint_trajectory: 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 0
#         nsecs:         0
#       frame_id: ''
#     joint_names: []
#     points: []
# planning_time: 0.024694407
