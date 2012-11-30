#include "ros/ros.h"
#include <Eigen/Dense>
#include <feature_extractor.hpp>
#include <graph_slam.hpp>
#include <robotpos.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/OccupancyGrid.h"
using namespace Eigen;
	ros::Publisher point_cloud_publisher_;
	ros::Publisher occupub;
//std::vector <Vector2d> odoms; //Odometry saver TODO somehow did not work on 1 system
std::vector <MatrixXd> Zs; //Z saver TODO Test if this works!
int t = 0;
int deltaT = 1;
MatrixXd u = MatrixXd::Zero(2, 1);
bool flag = false;
double speed;
double angular;
sensor_msgs::LaserScan::ConstPtr savescan;	
/*	Laserscancallback needed for feature extraction
--------------------------------------------------------------------------------------*/
void callback(const sensor_msgs::LaserScan::ConstPtr& msg) { // Always call graph slam for new laser readings
	if(!flag){
	savescan = msg;
	flag=true;
	}
}
/*	Robot Position function, values from Graphslam should be incorporated here
--------------------------------------------------------------------------------------*/
void rob_callback(const ros::TimerEvent&) { 
robotpos(0,0,0,0,0);	//current robot position in world is alway 0,0
}
/*		Velocity callback function, called when robot moves
--------------------------------------------------------------------------------------*/
void vel_callback(const geometry_msgs::Twist &msg) { 
	// Add new measurement matrix to z, n columns row 0 = range ; row 1 = angle in rad
	Zs.push_back(feature_extractor(savescan,point_cloud_publisher_,occupub));	
	//Declare Odometry here
	speed = sqrt(msg.linear.x*msg.linear.x+msg.linear.y*msg.linear.y);
	angular = msg.angular.z;
	Vector2d odometry = Vector2d::Zero(2, 1);
	odometry[0] = speed;
	odometry[1] = angular+=0.0000000001;	//small value to get results TODO find better solution
   	ROS_INFO_STREAM("Robot speed linear:"<< odometry[0]);
	ROS_INFO_STREAM("Robot speed angular:"<< odometry[1]);
	// Increment time t
	t += deltaT;
	// Expand old u and add new odometry
	MatrixXd newU(2, t);
	newU.block(0, 0, 2, u.cols()) = u;
	newU.block(0, t - 1, 2, 1) = odometry;
	u = newU;
	// Call the graph slam algorithm with unknown correspondences with odometry and measurement matrix + time deltaT
	MatrixXd mu = graph_slam(u, Zs, deltaT);
	std::cout << "mu = \n" << mu << std::endl;
	flag = false; 
}
int main(int argc, char **argv) {
	 
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	ros::Timer timer = n.createTimer(ros::Duration(1.0), rob_callback);
        ros::Subscriber laserSub = n.subscribe("base_scan", 100, callback);
	ros::Subscriber velSub = n.subscribe("cmd_vel", 100, vel_callback);
	point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud> ("/cloud", 100,false);
	occupub = n.advertise<nav_msgs::OccupancyGrid> ("/world", 100,false);
	ros::spin();
	return 0;
}

