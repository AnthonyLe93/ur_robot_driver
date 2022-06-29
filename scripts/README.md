# How to start up tas UR10-1

__Starting UR10-1__
roslaunch ur_robot_driver tas-ur10-1.launch
__Control the robot using MoveIt__
roslaunch ur_robot_driver tas-ur10-1.launch
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch
roslaunch ur10_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur10_moveit_config)/launch/moveit.rviz
rosrun ur_robot_driver UR10_1_control.py 
rosrun ur_robot_driver move_gazebo.py                 # testing with moveit
rosrun ur_robot_driver JointState_sub.py 
# Run each command in a seperate terminal windows within catkin_ws directory
[Using the ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md)
__MoveIt! with a simulated robot__
# Again, you can use MoveIt! to control the simulated robot.
# For setting up the MoveIt! nodes to allow motion planning run:

roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true

# For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

roslaunch ur10_moveit_config moveit_rviz.launch config:=true

# NOTE:
# As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:

roslaunch ur_gazebo ur10_bringup.launch
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
roslaunch ur10_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur10_moveit_config)/launch/moveit.rviz

# rqt_graph creates a dynamic graph of what's going on in the system
rosrun rqt_graph rqt_graph

