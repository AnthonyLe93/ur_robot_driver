#!/usr/bin/env python3
import rospy
import os
import sys
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode
import numpy as np


save_path = os.path.dirname(os.path.abspath(__file__)) + '/data/' #path to save data
print_JointStates = False
counter = 0
force_array = np.empty((0,1))
#print(save_path)

def joint_trajectory_callback(data):
    global print_JointStates
        
    data = data.goal.trajectory
    #rospy.loginfo(data.goal.trajectory)       
    print_JointStates = True
    
    
    
def joint_state_callback(joint_states):
    global print_JointStates
    global counter
    global force_array
    if print_JointStates:
        joints = joint_states.effort 
        #rospy.loginfo(joints)        
        counter += 1        
        print(counter)
        print(joints)
        force_array = np.append(force_array, joints)                   
        print_JointStates = False                    
          
    np.savetxt(save_path + 'joint_forces.txt', force_array)                    
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('JointState_sub', anonymous=True)
    rospy.Subscriber("/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal",
                     FollowJointTrajectoryActionGoal,
                     joint_trajectory_callback)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)   
       
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

     
        

if __name__ == '__main__':   
    try:
        listener()                                  
    except rospy.ROSInterruptException:
        pass
        
        
        
        
