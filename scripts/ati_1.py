#!/usr/bin/env python3
import rospy
import os
import sys
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import unittest
import actionlib

save_path = os.path.dirname(os.path.abspath(__file__)) + '/data/' #path to save data
print_JointStates = False
counter = 0
force_array = np.empty((0,1))
#print(save_path)

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]


def joint_trajectory_callback(data):
    global print_JointStates
        
    data = data.goal.trajectory
    #rospy.loginfo(data.goal.trajectory)       
    print_JointStates = True
    
    
                   
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.    
    
    rospy.init_node('ati_sub', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, ATI_loadcell_test)    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()   
     

def ATI_loadcell_test(joint_states):
    global force_array
    
    
    
    joints = joint_states.effort 
    #rospy.loginfo(joints)
    fx = joints[0]
    fy = joints[1]
    fz = joints[2]
    mx = joints[3]
    my = joints[4]
    mz = joints[5]
    #force_array = np.append(force_array, joints)
    #np.savetxt(save_path + 'ati_forces.txt', force_array)          
    if fx >= 290.0 or fy >= 290.0 or fz >= 580.0 or mx >= 10.0 or my >= 10.0 or mz >= 10.0: # fx >= 290.0 or fy >= 290.0 or fz >= 580.0 or mx >= 10.0 or my >= 10.0 or mz >= 10.0        
        rospy.loginfo("Abort!!!\n" "Max force/torque violation!!!.")
        print(f'fx: {fx}, fy: {fy}, fz: {fz}, mx: {mx}, my: {my}, mz: {mz}')
        set_robot_to_mode(RobotMode.POWER_OFF)

                          


def set_robot_to_mode(target_mode):
    timeout = rospy.Duration(5)
    set_mode_client = actionlib.SimpleActionClient('/ur_hardware_interface/set_mode', SetModeAction)
    if not set_mode_client.wait_for_server(timeout):
            fail(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))
    goal = SetModeGoal()
    goal.target_robot_mode = target_mode
    goal.play_program = True # we use headless mode during tests
    # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
    goal.stop_program = True

    set_mode_client.send_goal(goal)
    set_mode_client.wait_for_result()
    return set_mode_client.get_result().success
    
    
def send_joint_trajectory():
        """Creates a trajectory and sends it using the selected action server"""
        timeout = rospy.Duration(5)        
        joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )
               
        if not trajectory_client.wait_for_server(timeout):
            fail(
                "Could not reach controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired
        # The values are for each joint and in rad
        # position_list = [[0, -90, -90, 0, 0, 0]] - degree equivalent for easy reference
        position_list = [[-0.76, -1.71, -2, -0.81, 1.55, 0]]
        position_list.append([-1.91, -1.71, -2.1, -0.81, 1.55, 0])
        position_list.append([-1.547, -1.7, -2.38, 1.16, 1.55, 0]) # home position for tool exchange
        duration_list = [4.0, 8.0, 12.0]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        
        rospy.loginfo("Executing trajectory using the {}".format(joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))    

if __name__ == '__main__':   
    try:
        listener()                                  
    except rospy.ROSInterruptException:
        pass
