#!/usr/bin/env python3
import sys
import rospy
import copy, math


from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = 'CUBIE-GRIP'
robot_operational = True
counter = 0
JOINT_HOME = (-1.547, -1.7, -2.38, 1.16, 1.55, 0)


class UR10_move():

    def __init__(self):
        roscpp_initialize(sys.argv)        
        rospy.init_node('ur10_move', anonymous=True)

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #self.robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER) # adding gripper
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

    def move_home(self, joint_home):
          
        self.robot_arm.set_named_target("home")  # go to goal state. tool exchange state
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
        robot_state = self.robot_arm.get_current_pose()
        robot_angle = self.robot_arm.get_current_joint_values()

        print(robot_state)
        print(robot_angle)

        if joint_home != joint_goal:
            return True
        return False

    def poses(self):

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(-74.64)
        joint_goal[1] = math.radians(-83.47)
        joint_goal[2] = math.radians(-117.29)
        joint_goal[3] = math.radians(-67.70)
        joint_goal[4] = math.radians(88)
        joint_goal[5] = math.radians(0)
        self.robot_arm.remember_joint_values("ready_to_grip", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(-74.53)
        joint_goal[1] = math.radians(-99.68)
        joint_goal[2] = math.radians(-131.05)
        joint_goal[3] = math.radians(-37.72)
        joint_goal[4] = math.radians(87.93)
        joint_goal[5] = math.radians(180)
        self.robot_arm.remember_joint_values("move_gripper_down", joint_goal)  # go to goal state.

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(-74.64)
        joint_goal[1] = math.radians(-83.47)
        joint_goal[2] = math.radians(-117.29)
        joint_goal[3] = math.radians(-67.70)
        joint_goal[4] = math.radians(88)
        joint_goal[5] = math.radians(0)
        self.robot_arm.remember_joint_values("move_gripper_up", joint_goal)  # go to goal state.

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(-49.38)
        joint_goal[1] = math.radians(-92.61)
        joint_goal[2] = math.radians(-115.88)
        joint_goal[3] = math.radians(-61.88)
        joint_goal[4] = math.radians(87.76)
        joint_goal[5] = math.radians(90)
        self.robot_arm.remember_joint_values("move_gripper_across", joint_goal)  # go to goal state.

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(-49.32)
        joint_goal[1] = math.radians(-104.14)
        joint_goal[2] = math.radians(-124.10)
        joint_goal[3] = math.radians(-42.14)
        joint_goal[4] = math.radians(87.71)
        joint_goal[5] = math.radians(0)
        self.robot_arm.remember_joint_values("end_grip", joint_goal)  # go to goal state. Ready to grip

        robot_state = self.robot_arm.get_current_pose()
        joint_goal = self.robot_arm.get_current_joint_values()
        print(robot_state)
        print(joint_goal)
        goal_names = self.robot_arm.get_remembered_joint_values()
        print(f'goal_names = {goal_names}')
        return goal_names

    def move_grip(self):
        joint_goal = self.poses()
        goal = joint_goal.get('ready_to_grip')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== move plan go to grip home ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        rospy.sleep(1)

        goal = joint_goal.get('move_gripper_down')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== move gripper down and wait 5s ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        rospy.sleep(5)

        goal = joint_goal.get('move_gripper_up')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== move gripper up and wait 1s ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        rospy.sleep(1)

        goal = joint_goal.get('move_gripper_across')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== move gripper across and wait 1s ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        rospy.sleep(1)

        goal = joint_goal.get('end_grip')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== move gripper down, rotate end effector 90 degree and wait 5s ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        rospy.sleep(5)


if __name__ == '__main__':

    robot = UR10_move()
    robot.__init__()
    robot.display_trajectory()
    try:
        runTime = int(input('''Enter how many times you want the robot to run through the pick and place sequence: 
Or enter '0' to enter tool exchange mode: '''))
    except ValueError:
        print('Invalid input!!!\n Please enter a integer!')
        sys.exit(0)

    while robot_operational:
        if runTime == 0:
            robot.move_home(JOINT_HOME)
            if robot.move_home(JOINT_HOME):
                print('The arm is at tool exchange mode')
                break
            else:
                print('The arm is at another position')
        else:
            robot.move_grip()
            #rospy.spin()
            #roscpp_shutdown()
            counter += 1
            print(counter)
            if counter == runTime:
                robot_operational = False




    
    
    
    
    
