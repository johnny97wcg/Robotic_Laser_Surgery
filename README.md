# Robotic_Laser_Surgery project

#start ros core
roscore

#launch moveit planning interface
roslaunch abb_irb120_moveit_config abb_irb120_moveit_planning_execution.launch  robot_ip:="192.168.125.1" 

#run moveit node
rosrun moveit move_group_interface
 
#start matlab
opengl render
launch matlab from terminal 
run KinematicsServer
