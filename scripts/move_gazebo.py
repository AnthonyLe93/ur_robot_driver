#!/usr/bin/env python3
import sys
import rospy
import copy, math
from math import pi

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"

class TestMove():

    def __init__(self):
        roscpp_initialize(sys.argv)        
        rospy.init_node('ur10_move',anonymous=True)

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(5)
        self.robot_arm.set_num_planning_attempts(5)

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True) 
        
    def display_trajectory(self):
        display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        DisplayTrajectory,
        queue_size=20,
        )
        
        # We can get the name of the reference frame for this robot:
        planning_frame = self.robot_arm.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.robot_arm.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot_cmd.get_group_names()
        print("============ Available Planning Groups:", self.robot_cmd.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot_cmd.get_current_state())
        print("")           

    def move_code(self):
          
        self.robot_arm.set_named_target("home")  #go to goal state. tool exchange state
        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = -1.547
        joint_goal[1] = -1.7
        joint_goal[2] = -2.38
        joint_goal[3] = 1.16
        joint_goal[4] = 1.55
        joint_goal[5] = 0
        
        self.robot_arm.go(joint_goal, wait=True)
        print("====== move plan go to home (tool exchange) ======") 
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()       
        rospy.sleep(1)        
        
        print("====== move plan go to gripping pose ======")
        self.robot_arm.set_named_target("grip-home")  #go to goal state.
        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = -1.29
        joint_goal[1] = -1.57
        joint_goal[2] = -2.22
        joint_goal[3] = -0.92
        joint_goal[4] = 1.55
        joint_goal[5] = 0
        
        self.robot_arm.go(wait=True)
        print("====== move plan go to grip-home ======")        
        rospy.sleep(1)       

#        robot_arm.set_named_target("up")
#        robot_arm.go(wait=True)

        robot_state = self.robot_arm.get_current_pose()
        robot_angle = self.robot_arm.get_current_joint_values()

        print(robot_state)

        
if __name__=='__main__':
    tm = TestMove()
    tm.__init__()
    tm.display_trajectory()
    tm.move_code()

    rospy.spin()
    roscpp_shutdown()
    
    
    
    
    
