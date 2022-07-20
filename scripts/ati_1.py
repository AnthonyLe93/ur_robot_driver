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



class ATI_ft():

    def __init__(self):
        self.save_path = os.path.dirname(os.path.abspath(__file__)) + '/data/'  # path to save data
        self.print_JointStates = False
        self.counter = 0
        self.force_array = np.empty((0, 1))

    def joint_trajectory_callback(self, data):

        data = data.goal.trajectory
        # rospy.loginfo(data.goal.trajectory)
        self.print_JointStates = True

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('ati_sub', anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.ATI_loadcell_test)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def ATI_loadcell_test(self, joint_states):

        joints = joint_states.effort

        rospy.loginfo(joint_states.position)
        fx = joints[0]
        fy = joints[1]
        fz = joints[2]
        mx = joints[3]
        my = joints[4]
        mz = joints[5]
        # force_array = np.append(force_array, joints)
        # np.savetxt(save_path + 'ati_forces.txt', force_array)
        # right now set to the max torque of the ati mini45 ft sensor
        # the end effector of the ur10 has a max torque of 56Nm
        if fx >= 10 or fy >= 10 or fz >= 10 or mx >= 10 or my >= 10 or mz >= 10: # fx >= 290.0 or fy >= 290.0 or fz >= 580.0 or mx >= 10.0 or my >= 10.0 or mz >= 10.0
            rospy.loginfo("Abort!!!\n" "Max force/torque violation!!!.")
            print(f'fx: {fx}, fy: {fy}, fz: {fz}, mx: {mx}, my: {my}, mz: {mz}')
            self.set_robot_to_mode(RobotMode.POWER_OFF)

        print(f'fx: {fx}, fy: {fy}, fz: {fz}, mx: {mx}, my: {my}, mz: {mz}')

    def set_robot_to_mode(self, target_mode):
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


if __name__ == '__main__':
    ATI = ATI_ft()
    ATI.listener()


