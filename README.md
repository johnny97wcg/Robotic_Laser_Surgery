# Robotic_Laser_Surgery Project
This is an undergraduate summer research project at WPI with professor Loris Fichera, on the subject of robotic control of laser and study of laser incisions on agar geletins. This is a preliminary study as part of a joint project involving infered thermal imaging, minimally invasive continuum endoscope and etc. For more information, please visit [WPI COMETLAB](https://www.wpicometlab.com/).


## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See [Deployment](https://github.com/johnny97wcg/Robotic_Laser_Surgery#deployment) for notes on how to deploy the project on a live system. For better understanding of this project, refer to [Repository Overview](https://github.com/johnny97wcg/Robotic_Laser_Surgery#repository-overview).

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
### Repository Overview
Here is the outline of this repository and notes on how to replicate some of the folder contents.

- **/Others** folder is for online backup and displaying image only, no need to clone for project development.

- **/src/abb_driver** folder includes ABB ROS drivers that enables communication with the ABB robot through ROS, see [abb_drivers](http://wiki.ros.org/abb_driver) for more information or version updates. 

- **/src/irb120_description** includes the STL mashes for the robot model and URDF file that defiles the connectin among robot links. [Xacro](http://wiki.ros.org/xacro) was used to simplify the coding process and it can use terminal commands such as	 ` $ rosrun xacro xacro --inorder model.xacro > model.urdf`

- **/src/abb_irb120_moveit_config** is a MoveIt! configuration file auto-generated by launching the [moveit_setup_assistant](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) wizard.

- **/src/matlab_codes** folder hosts all the matlab functions and scripts, and this is the folder that needs to be set as 'current folder' in matlab.

- **/src/moveit** folder includes the c++ file for moveitNavigation node, this is the program that communicates with matlab and send goals to the robot joint trajectory. 

- **/src/Laser_Control_Module** serial communication packet that enable laser activation from computer, given by Benjamin Gillette. The laser_control_model subfolder shows the code stored in the arduino, and the pyarduino subfolder includes python scripts for testing. 

### Initial Setup
clone this git repo locally, setup catkin workspace, and source the path

	$ git clone https://github.com/johnny97wcg/Robotic_Laser_Surgery
	$ catkin_make 
	$ source devel/setup.bash	
#### on Windows computer, download RobotStudio and load ROS packages into robot controller
**Note** This process has already been done on the IRB120 robot in AIM lab, no need to repeat if using the same robot.
Otherwise, please follow the ROS Tutorials: [Install RAPID Files](http://wiki.ros.org/abb/Tutorials/RobotStudio), [Install ROS Server](http://wiki.ros.org/abb/Tutorials/InstallServer), [Running ROS Server](http://wiki.ros.org/abb/Tutorials/RunServer).
Afterwards the robot controller should have a file structure similar to the one shown in the [screenshot](/Others/RobotStudio_Screenshot.PNG) (also viewable on the flexpendant). 

If the files get deleted accidently, here is a link to the [system backup](/Others/System1_BACKUP_2018-06-20). Use the 'restore from backup' function in RobotStudio to reload the files onto the controller.


## Running the program
Here are the steps for launching the program

### 1. Start `roscore`
It is recommended to define ROS_HOSTNAME and ROS_MASTER_URI explicitly so that nodes can locate their master.
	
	Linux terminal:
	$ roscore
	$ export ROS_MASTER_URI=YOUR_IP:11311
	$ export ROS_HOSTNAME=YOUR_HOSTNAME
	MATLAB command line:
	$ setenv(ROS_MASTER_URI, YOUR_IP:11311)
	$ setenv(ROS_HOSTNAME, YOUR_HOSTNAME)	
	
### 2. Launch Moveit_Planning_Interface
By default robot ip is 192.168.125.1, adjust accordingly based on your connection. Set argument 'rviz' to true to enable moveit planner GUI in rviz, only if needed. **Note** robot needs to be powered on from this step onward.

	$ roslaunch abb_irb120_moveit_config abb_irb120_moveit_planning_execution.launch  robot_ip:="192.168.125.1" rviz:="true"
**Note** rviz moveit GUI allows interation of robot motion in simulation through simple drag & rotate, it is capable of path planning and sending mothion commands to the physical robot; however, the GUI is not helpful for our purpose of percise control. Thus creating a moveit_node is necessary to send setpoints and process trajectory.
	
### 3. Start MATLAB
launch matlab from terminal (after adding it to the search path). This is recomended over shorcut icon since matlab tends to crash occationally during the project development phase.
	
	$ matlab
in class file Kinematics.m, change the absolute path on line 10 to the correct path to urdf file on your host machine.

	`obj.robot = importrobot('/home/YOUR_HOSTNAME/YOUR_CATKIN_WORKSPACE/src/irb120_description/urdf/abb_irb120.urdf');
run KinematicsServer node (in matlab command line, change folder or add to path if necessary)
	
	$ KinematicsServer


### 4. Run Moveit_node
	$ rosrun moveit move_group_interface
 
## Deployment
Here are some tips on operating the ABB robot on the flexpendent

### Robot Calibration
Due to some unknown battery related failure, our IRB120 robot requires calibration everytime after reboot, thus it is convenient to always have the robot return to homeposition before power off. To calibrate, simply go to the Calibration tab on the flexpendent and click "Update Rev Conuters". **make sure the robot is at the homeposition**, and click update. 

### Robot Connection and Activation
Once the power is turned on and robot controller is connected to the desktop through ethernet cable, a 'connected' message should showup on both end (after completing step 2 above). And after executing the path in step 4, the robot controller should receive points in the trajectory. In manual mode (recommended), hold the Enable switch and press the Play button to start the motion. Each time after release of the Enable button/E-stop, the Play button has to be pressed again to continue the motion. In autonomous mode, press the motor on and play button to activate the robot. 

# Author
##### Chenggu Wang 
Studying in BS program in Robotic Enginnering and Mechanical Engineering at Worcester Polytechnic Institute (class of 2019). Area of research is ROS C++ programming, robotic control and serial communication.
# Co-Author
##### Samantha Moriarty
Studying in BS program in Biomedical Engineering at Worcester Polytechnic Institute (class of 2019). Area of research is fabrication and analysis of tissues for laser experiment
