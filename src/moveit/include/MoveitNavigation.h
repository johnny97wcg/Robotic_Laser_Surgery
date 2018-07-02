#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
using namespace Eigen;
using namespace std;

bool flag;
std_msgs::Float64 joint1_goal;
std_msgs::Float64 joint2_goal;
std_msgs::Float64 joint3_goal;
std_msgs::Float64 joint4_goal;
std_msgs::Float64 joint5_goal;
std_msgs::Float64 joint6_goal;
geometry_msgs::Point A1, A2, A3, A4;
geometry_msgs::Point B1, B2, B3, B4;
geometry_msgs::Point C1, C2, C3, C4;
vector<geometry_msgs::Point> pointArray;
void initPoints();
void setJointGoal(string mode, vector<double> &joint_group_positions);

class MoveitNavigation
{
private:
	ros::Publisher local_goal_pub;
	ros::Subscriber joint1_sub;
	ros::Subscriber joint2_sub;
	ros::Subscriber joint3_sub;
	ros::Subscriber joint4_sub;
	ros::Subscriber joint5_sub;
	ros::Subscriber joint6_sub;
public:
	MoveitNavigation(ros::NodeHandle n)
	{
		local_goal_pub = n.advertise<geometry_msgs::Point>("moveit_goal",1);
		joint1_sub = n.subscribe("/abb_irb120/joint_1_position_controller/command",1,&MoveitNavigation::Joint1Callback, this);
		joint2_sub = n.subscribe("/abb_irb120/joint_2_position_controller/command",1,&MoveitNavigation::Joint2Callback, this);
		joint3_sub = n.subscribe("/abb_irb120/joint_3_position_controller/command",1,&MoveitNavigation::Joint3Callback, this);
		joint4_sub = n.subscribe("/abb_irb120/joint_4_position_controller/command",1,&MoveitNavigation::Joint4Callback, this);
		joint5_sub = n.subscribe("/abb_irb120/joint_5_position_controller/command",1,&MoveitNavigation::Joint5Callback, this);
		joint6_sub = n.subscribe("/abb_irb120/joint_6_position_controller/command",1,&MoveitNavigation::Joint6Callback, this);
	}
	void Joint1Callback(const std_msgs::Float64 joint1_msg);
	void Joint2Callback(const std_msgs::Float64 joint2_msg);
	void Joint3Callback(const std_msgs::Float64 joint3_msg);
	void Joint4Callback(const std_msgs::Float64 joint4_msg);
	void Joint5Callback(const std_msgs::Float64 joint5_msg);
	void Joint6Callback(const std_msgs::Float64 joint6_msg);
	void pub_local_goal(double x, double y, double z);
	void pub_point (geometry_msgs::Point point, double dx, double dy, double dz);
	geometry_msgs::Pose ConstructPose(VectorXd position, VectorXd orientation);

};
