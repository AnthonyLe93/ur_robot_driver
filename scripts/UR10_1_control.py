#!/usr/bin/env python3

import sys
import roslib
roslib.load_manifest('ur_robot_driver')
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

from math import radians as rad

robot = True

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

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        rospy.init_node("UR10_move")

        #timeout = rospy.Duration(0.5)
        
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        self.robot = robot

    def send_joint_trajectory(self):
        """Creates a trajectory and sends it using the selected action server"""
               
        
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )
        
        timeout = rospy.Duration(5)        
        if not trajectory_client.wait_for_server(timeout):
            self.fail(
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

        
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def send_cartesian_trajectory(self):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        timeout = rospy.Duration(5) 
        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )
        if not trajectory_client.wait_for_server(timeout):
            self.fail(
                "Could not reach controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        # The following list are arbitrary positions
        # Change to your own needs if desired
        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, -0.1, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, 0.3, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, 0.3, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
        ]
        duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        
        rospy.loginfo(
            "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)
        )
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    
	
    def move_arm(self):
        """Creates a trajectory and sends it using the selected action server"""
        # make sure the correct controller is loaded and activated       
        timeout = rospy.Duration(5)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        ) 
        if not trajectory_client.wait_for_server(timeout):
            self.fail(
                "Could not reach controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))
                    
        # The following list are arbitrary positions
        # Change to your own needs if desired
        # The values are for each joint and in rad
        # position_list = [[0, -90, -90, 0, 0, 0]] - degree equivalent for easy reference
        position_list = [[-1.29, -1.57, -2.22, -0.92, 1.55, 0]] # ready to grip
        position_list.append([-1.33, -1.909, -2.33, -0.38, 1.54, 0]) # goes down and grip        
        duration_list = [3.0, 6.0]
        
        
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        for i, position in enumerate(position_list):
           try:                         
              point = JointTrajectoryPoint()
              point.positions = position
              point.time_from_start = rospy.Duration(duration_list[i])
              goal.trajectory.points.append(point)
           except:
              rospy.logwarn("Could not parse pose \"%s\" from the param server:", i);
              rospy.logwarn(sys.exc_info())   
            
            
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code)) 
        rospy.sleep(5) # wait 10s - gripper closing here
        
        
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired
        # The values are for each joint and in rad
        # position_list = [[0, -90, -90, 0, 0, 0]] - degree equivalent for easy reference      
        position_list = [[-1.29, -1.58, -2.23, -0.90, 1.55, 0]]  # go up
        position_list.append([-0.94, -1.70, -2.16, -0.60, 1.54, 0])       
        position_list.append([-0.89, -1.97, -2.21, -0.43, 1.54, 0]) # go to other position and release        
        duration_list = [4.0, 8.0, 12.0]
        
        for i, position in enumerate(position_list):
           try:                         
              point = JointTrajectoryPoint()
              point.positions = position
              point.time_from_start = rospy.Duration(duration_list[i])
              goal.trajectory.points.append(point)
           except:
              rospy.logwarn("Could not parse pose \"%s\" from the param server:", i);
              rospy.logwarn(sys.exc_info())   
            

        
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
        rospy.sleep(5) # wait 5s - gripper opening here
        
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired
        # The values are for each joint and in rad
        # position_list = [[0, -90, -90, 0, 0, 0]] - degree equivalent for easy reference             
        position_list = [[-0.90, -1.79, -2.2, -0.46, 1.55, 0]] 
        position_list.append([-1.29, -1.57, -2.22, -0.92, 1.55, 0]) # go back to ready to grip  
        duration_list = [4.0,8.0]
        
        for i, position in enumerate(position_list):
           try:                         
              point = JointTrajectoryPoint()
              point.positions = position
              point.time_from_start = rospy.Duration(duration_list[i])
              goal.trajectory.points.append(point)
           except:
              rospy.logwarn("Could not parse pose \"%s\" from the param server:", i);
              rospy.logwarn(sys.exc_info())   
            
        
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
        rospy.sleep(5) # wait 5s

    def pick_operation_mode(self):        
        # Ask the user for mode of operations              
        input_str = input(
                "Please confirm the robot operation mode.\n"                
                "Please type 't' for tool exchange or 'o' for operation or 'a' to abort: "
            )        
        if input_str == 't':
            rospy.loginfo("Robot enters tool exchange mode")
            self.send_joint_trajectory()
        elif input_str == 'o':
            rospy.loginfo("Robot enters operational mode")
            self.move_arm()
        elif input_str == 'a':
            robot = False
            rospy.loginfo("Abort!!!\n" "Exiting as requested by user.")
            sys.exit(0)   
        else:    
            input_str == 'a'       
            rospy.loginfo("Exiting as requested by user.")
            sys.exit(0)   




if __name__ == "__main__":             
    client = TrajectoryClient()
    counter = 0    
    #client.pick_operation_mode()
    while client.robot:                          
        client.move_arm()
        #client.pick_operation_mode()
        #client.send_joint_trajectory()  
        print('counter = ', counter)              
        counter = counter + 1   

            
            
            
            
