#include "ros/ros.h"
#include <Eigen/Dense>
#include <feature_extractor.hpp>
#include <graph_slam.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
using namespace Eigen;

int t = 0;
int deltaT = 1;
MatrixXd u = MatrixXd::Zero(2, 1);
std::vector<MatrixXd> z;
//const ros::TimerEvent&
//const sensor_msgs::LaserScan::ConstPtr& msg
void callback(const ros::TimerEvent&) { // Always call graph slam for new laser readings
	//feature_extractor(msg);
	// Increment time t
	t += deltaT;
	//std::cout << "t = " << t << std::endl;

	// Generate random odometry, will be replaced by actual odometry
	Vector2d odometry = Vector2d::Random(2, 1);

	// Expand old u and add new odometry
	MatrixXd newU(2, t);
	newU.block(0, 0, 2, u.cols()) = u;
	newU.block(0, t - 1, 2, 1) = odometry;
	u = newU;

	// Generate random measurement, will be replaced by actual measurements
	MatrixXd newMeasurement = MatrixXd::Random(3, 2); // We assume for every measurement two features are observed
	z.push_back(newMeasurement);

	// Call the graph slam algorithm with unknown correspondences
	MatrixXd mu = graph_slam(u, z, deltaT);
	std::cout << "last mu = \n" << mu << std::endl;
};

int main(int argc, char **argv) {
	 
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	ros::Timer timer = n.createTimer(ros::Duration(1.0), callback);
	//ros::Subscriber laserSub = n.subscribe("base_scan", 100, callback);
	ros::spin();
	return 0;
}

