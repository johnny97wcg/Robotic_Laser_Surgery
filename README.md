# Robotic_Laser_Surgery Project
project description here

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites
```
ROBOT: ABB IRB120 Robot, IRC5 Controller
OS: Ubuntu 16.04 LTS, Windows (only for initializing ABB robot in RobotStudio)
ROS: Kinetic Kame
MATLAB: R2018a
```

### Installing
```
Eigen : http://eigen.tuxfamily.org/index.php?title=Main_Page
ros-controllers : $ sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
MoveIt!: $ sudo apt-get install ros-kinetic-moveit
```

## Running the program

#### clone this git repo locally and setup catkin workspace
	$ git clone https://github.com/johnny97wcg/Robotic_Laser_Surgery
	$ catkin_make
# 2. start ros core 
	$ roscore
# 3. launch moveit planning interface
	by default robot ip is 192.168.125.1, adjust accordingly
	set argument 'rviz' to true to enable moveit planner GUI in rviz, only if needed 
	$ roslaunch abb_irb120_moveit_config abb_irb120_moveit_planning_execution.launch  robot_ip:="192.168.125.1" rviz:="true"
	
# 4. start matlab
	# launch matlab from terminal 
	$ matlab
	# run KinematicsServer node
	$ (in matlab command line) KinematicsServer

# 4. run moveit node
	$ rosrun moveit move_group_interface
 
