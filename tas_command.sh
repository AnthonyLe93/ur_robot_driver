#!/bin/bash
# cd $HOME/catkin_ws
xterm -hold -e "roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch" &
xterm -hold -e "roslaunch ur10_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur10_moveit_config)/launch/moveit.rviz " &
xterm -hold -e "rosrun TAS_python_msg ati_u6_pub.py" &
xterm -hold -e "rosrun TAS_python_msg ati_sub.py" &
xterm -hold -e "rqt_plot /ati_readings" &
