// created by Chenggu Wang on 6.25.18, references to ankur's robot dynamics project
//moveit c++ tutorial: https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp

//Move It header files
#include<MoveitNavigation.h>

void initPoints(vector<geometry_msgs::Point> &pointArray)
{
	A1.x = 110; A1.y = 20; A1.z = 12;
	A2.x = 80; A2.y = 20; A2.z = 12;
	A3.x = 50; A3.y = 20; A3.z = 12;
	A4.x = 20; A4.y = 20; A4.z = 12;
	B1.x = 110; B1.y = 50; B1.z = 12;
	B2.x = 80; B2.y = 50; B2.z = 12;
	B3.x = 50; B3.y = 50; B3.z = 12;
	B4.x = 20; B4.y = 50; B4.z = 12;
	C1.x = 110; C1.y = 80; C1.z = 12;
	C2.x = 80; C2.y = 80; C2.z = 12;
	C3.x = 50; C3.y = 80; C3.z = 12;
	C4.x = 20; C4.y = 80; C4.z = 12;
	pointArray.push_back(A1);
	pointArray.push_back(A2);
	pointArray.push_back(A3);
	pointArray.push_back(A4);
	pointArray.push_back(B1);
	pointArray.push_back(B2);
	pointArray.push_back(B3);
	pointArray.push_back(B4);
	pointArray.push_back(C1);
	pointArray.push_back(C2);
	pointArray.push_back(C3);
	pointArray.push_back(C4);
}

// subscribe callback functions for each joint
void MoveitNavigation::Joint1Callback(const std_msgs::Float64 joint1_msg)
{
	joint1_goal.data = joint1_msg.data;
}void MoveitNavigation::Joint2Callback(const std_msgs::Float64 joint2_msg)
{
	joint2_goal.data = joint2_msg.data;
}void MoveitNavigation::Joint3Callback(const std_msgs::Float64 joint3_msg)
{
	joint3_goal.data = joint3_msg.data;
}void MoveitNavigation::Joint4Callback(const std_msgs::Float64 joint4_msg)
{
	joint4_goal.data = joint4_msg.data;
}void MoveitNavigation::Joint5Callback(const std_msgs::Float64 joint5_msg)
{
	joint5_goal.data = joint5_msg.data;
}void MoveitNavigation::Joint6Callback(const std_msgs::Float64 joint6_msg)
{
	joint6_goal.data = joint6_msg.data;
	flag = false; //after all joint goals received, reset flag value
	cout << "joint goals received" << endl;
}

// publish a point in the the workobject's reference frame
void MoveitNavigation::pub_local_goal(double x, double y, double z)
{
		geometry_msgs::Point pt_msg;
		pt_msg.x = x;
		pt_msg.y = y;
		pt_msg.z = z;
		local_goal_pub.publish(pt_msg);
		cout << "point sent";
}

//publish a predefined point to matlab for IK calculation
void MoveitNavigation::pub_point (geometry_msgs::Point point, double dx, double dy, double dz)
{
	point.x += dx;
	point.y += dy;
	point.z += dz;
	local_goal_pub.publish(point);
	cout << "point sent" << endl;
}

// choose the motion goal as home(all zeros), or the subscribed goal from matlab
void setJointGoal(string mode, vector<double> &joint_group_positions)
{
	if (mode.compare("home") == 0)
	{
		joint_group_positions[0] = 0;
		joint_group_positions[1] = 0;
		joint_group_positions[2] = 0;
		joint_group_positions[3] = 0;
		joint_group_positions[4] = 0;
		joint_group_positions[5] = 0;
	}
	else if (mode.compare("goal") == 0)
	{
		joint_group_positions[0] = joint1_goal.data;
		joint_group_positions[1] = joint2_goal.data;
		joint_group_positions[2] = joint3_goal.data;
		joint_group_positions[3] = joint4_goal.data;
		joint_group_positions[4] = joint5_goal.data;
		joint_group_positions[5] = joint6_goal.data;
	}
	else
	{
		cout << "mode not recognized, please type 'home' for home position or 'goal' for joint space goal." << endl;
	}
}

void MoveitNavigation::laserOn(void)
{
	laser.data = "on";
	cout << "laser Activated" << endl;
	arduino_pub.publish(laser);
}

void MoveitNavigation::laserOff(void)
{
	laser.data = "off";
	cout << "laser Deactivated" << endl;
	arduino_pub.publish(laser);
}

double vectorAbs(vector<double> v1, vector<double> v2)
{
	int l1 = v1.size();
	int l2 = v2.size();
	double Abs = 0;
	if (l1 != l2)
	{
		cout<<"error: vector size mismatch"<<endl;
		return 999;
	}
	else // compare each element in the vector and calculate the sum of absolute difference
	{
		for (int i=0; i<l1; i++)
		{
			Abs += abs(v1[i]-v2[i]);
		}
	}
	return Abs/l1;
}

void printVector(vector<double> v)
{
	int l = v.size();
	cout<<"printing vector: ";
	for (int i=0; i<l; i++)
	{
		cout<<v[i]<<", ";
	}
	cout<<endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_group_interface");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	ros::Rate loop_rate(1);
	spinner.start();
  MoveitNavigation nav_obj(node_handle);
	initPoints(pointArray);

	//select planning group (defined in moveit_setup_assistant)
	static const string PLANNING_GROUP = "manipulator";
	//construct move_group object
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	//for add and remove colliioin objecs in our virtual scene path planning
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// Raw pointers are frequently used to refer to the planning group for improved performance.
	const robot_state::JointModelGroup * joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	// initialize start state to be robot's current state, and create a plan object
	move_group.setStartStateToCurrentState();
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	// current state pointer
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	//initialize joint_group_positions
	vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	/*
	// //Visualization in RViz
	// namespace rvt = rviz_visual_tools;
	// moveit_visual_tools::MoveItVisualTools visual_tools("/world");
	// visual_tools.deleteAllMarkers();
	// // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	// Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	// text_pose.translation().z() = 1.75;
	// visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
	// // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	// visual_tools.trigger();
	// // We can print the name of the reference frame for this robot.
  // ROS_INFO_NAMED("abb_irb120", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	// // Start the demo
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
	*/

	char input;
	int counter;
  counter = 0;
	cout<< "waiting for keyboard input (y/n)" <<endl;
	while(cin >> input)
	{
		cout << "input :" << input << endl;
		if (input == 'n' || counter >= 120)
		{
			break;
		}

		else if (input == 'y' && counter < 120)
		{
			cout << "moving to next location" << endl;
			// setting point goal in the workobject	reference frame
			// 5 passes per row, left->right
				if (counter%2 == 0) // 1
				{
					nav_obj.pub_point(pointArray[counter/10],9,0,5); // left
				}
				else // 2
				{
					nav_obj.pub_point(pointArray[counter/10],-8,0,5); // right
				}

			flag = true;
			while (flag) // wait for matlab
			{
				cout << " waiting for matlab "<< endl;
				ros::spinOnce();
				loop_rate.sleep();
			}

			// sending joint Angles
			joint_start = move_group.getCurrentJointValues(); // starting angle
			setJointGoal("goal", joint_group_positions); // helper function that sets the joint space goal
			move_group.setJointValueTarget(joint_group_positions);
			if (counter == 1 || counter == 41 || counter == 81) //set speed for each section
			{
				// restrict the max speed and acceleation for short dist motion (1% of actual max)
				move_group.setMaxAccelerationScalingFactor(0.01);
				move_group.setMaxVelocityScalingFactor(0.01);
			}
			else if (counter == 40 || counter == 80)
			{
				// restrict the max speed and acceleation for long dist motion (10% of actual max)
				move_group.setMaxAccelerationScalingFactor(0.1);
				move_group.setMaxVelocityScalingFactor(0.1);
			}

			moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
			ROS_INFO_NAMED("Visualizing plan 1 (joint space goal) %s", success ? "SUCCESS":"FAILED");
			if (counter%2 == 1)
			{
				//printVector(joint_start);
				//printVector(joint_group_positions);
				move_group.asyncExecute(my_plan); // non-blocking execute

				do{
					joint_current = move_group.getCurrentJointValues();
					Abs = vectorAbs(joint_current,joint_start);
					cout << "absolute difference: " << Abs << endl;
				}while(Abs<1E-6); // exit after the motion started "by enoungh distance"
				nav_obj.laserOn(); // activate the laser

				do{
					joint_current = move_group.getCurrentJointValues();
					Abs = vectorAbs(joint_current,joint_group_positions);
					cout << "absolute difference: " << Abs << endl;
				}while(Abs>1E-3); // exit after reached the goal "within the tolerance"

				nav_obj.laserOff();

				// nav_obj.laserOn(); // activate the laser for 1 sec
				// move_group.execute(my_plan);
				// nav_obj.laserOff();
			}

			else
			{
				move_group.execute(my_plan);
			}
			counter ++;
			cout << "motion "<< counter <<" finished, waiting for next input"<<endl;
		}

		else
		{
			cout << "unrecognized input, press 'y' to continue or 'n' to stop" << endl ;
		}
	}

	// // move to set location
	// //define x,y,z coordinate of the goal in local frame (workobject)
	// double x_goal, y_goal, z_goal;
	// x_goal = 50;
	// y_goal = 50;
	// z_goal = 100;
	// nav_obj.pub_local_goal(x_goal, y_goal, z_goal);
	//
	// //move to predefined location with modification
	// double dx, dy, dz;
	// dx = -10;
	// dy = 0;
	// dz = 5;
	// nav_obj.pub_point(A1,dx,dy,dz);

	/*
	// // Visualizing plans
  // // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED("abb_irb120", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  */

	// move_group.setRandomTarget();
	// moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
	// ROS_INFO_NAMED("Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS":"FAILED");
	// moveit_msgs::RobotTrajectory traj;
	// traj = my_plan.trajectory_;
	// cout << "trajectory \n" << traj << endl;


	// return to home position before shutdown
	cout << "returning to home position" << endl;
	setJointGoal("home", joint_group_positions);
	move_group.setJointValueTarget(joint_group_positions);
	// reset the max speed and acceleation (100% of actual max)
	move_group.setMaxAccelerationScalingFactor(1.0);
	move_group.setMaxVelocityScalingFactor(1.0);
	moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
	ROS_INFO_NAMED("Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS":"FAILED");
	move_group.execute(my_plan);
	sleep(5);

	ros::shutdown();
	return 0;
}
