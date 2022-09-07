# How to start up tas UR10-1
Dependencies - clone this repo into the Universal_Robots_ROS_Driver
```bash
git clone git@github.com:AnthonyLe93/TAS_python_msg.git
```
## __Starting UR10-1__
might need to run this command for every terminal spawn.
```bash
source devel/setup.bash
roslaunch ur_robot_driver tas-ur10-1.launch
```
## __Control the robot using MoveIt__
```bash
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch
roslaunch ur10_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur10_moveit_config)/launch/moveit.rviz               
rosrun TAS_python_msg ati_u6_pub.py
rosrun TAS_python_msg ati_sub.py
rosrun ur_robot_driver ur10_movit_control.py  
``` 
### __Optional script to control robot using action service__
```bash
rosrun ur_robot_driver UR10_1_control.py 
```
### __Using rqt_plot to plot the ati f/t sensor readings__
```bash
rqt_plot /ati_readings
```
### _Run each command in a seperate terminal windows within catkin_ws directory_
### [Using the ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md)
## __MoveIt! with a simulated robot__
### Again, you can use MoveIt! to control the simulated robot.
### For setting up the MoveIt! nodes to allow motion planning run:
```bash
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
```
## For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
```bash
roslaunch ur10_moveit_config moveit_rviz.launch config:=true
```
### NOTE:
### _As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi],there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:_
```bash
roslaunch ur_gazebo ur10_bringup.launch
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
roslaunch ur10_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur10_moveit_config)/launch/moveit.rviz
```
## rqt_graph creates a dynamic graph of what's going on in the system
```bash
rosrun rqt_graph rqt_graph
```
