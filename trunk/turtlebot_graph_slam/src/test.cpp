#include "ros/ros.h"
#include <Eigen/Dense>
#include <graph_slam.hpp>
using namespace Eigen;

int t = 0;
int deltaT = 1;
MatrixXd u = MatrixXd::Zero(2, 1);
std::vector<MatrixXd> z;

void callback(const ros::TimerEvent&) {

	// Increment time t
	t += deltaT;
	std::cout << "t = " << t << std::endl;

	// Generate random odometry, will be replaced by actual odometry
	Vector2d odometry = Vector2d::Random(2, 1);

	// Expand old u and add new odometry
	MatrixXd newU(2, t);
	newU.block(0, 0, 2, u.cols()) = u;
	newU.block(0, t - 1, 2, 1) = odometry;
	u = newU;

	// Generate random measurement, will be replaced by actual measurements
	MatrixXd newMeasurement = MatrixXd::Random(3, 10);
	z.push_back(newMeasurement);

	// Call the graph slam algorithm with unknown correspondences
	MatrixXd mu = graph_slam(u, z, deltaT);
	std::cout << "mu = \n" << mu << std::endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	ros::Timer timer = n.createTimer(ros::Duration(1.0), callback);
	ros::spin();
	return 0;
}

