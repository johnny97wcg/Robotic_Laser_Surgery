# Robotic_Laser_Surgery Project
This is an undergraduate summer research project at WPI with professor Loris Fichera, on the subject of robotic control of laser and study of laser incisions on agar gels. This is a preliminary study as part of a joint project involving infered thermal imaging, minimally invasive continuum endoscope and etc. For more information, please visit ![WPI COMETLAB](https://www.wpicometlab.com/).


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
MoveIt!: $ sudo apt-get install ros-kinetic-moveit
```

### Initial Setup
#### clone this git repo locally, setup catkin workspace, and source the path
	$ git clone https://github.com/johnny97wcg/Robotic_Laser_Surgery
	$ catkin_make 
	$ source devel/setup.bash
	
#### on Windows computer, download RobotStudio and load ROS packages into robot controller
**Note** this process has already been done on the ABB IRB robot in the AIM lab, no need to repeat if using the same robot.
Otherwise, please follow the ROS Tutorials: ![Install RAPID Files](http://wiki.ros.org/abb/Tutorials/RobotStudio), ![Install ROS Server](http://wiki.ros.org/abb/Tutorials/InstallServer), ![Running ROS Server](http://wiki.ros.org/abb/Tutorials/RunServer).
Afterwards the robot controller should have a file structure similar to the one shown in the screenshot below (also viewable on the flexpendant). 

If the files are deleted accidently, here is a link to the ![backup file](INSERT LINK HERE). 


## Running the program

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
 
