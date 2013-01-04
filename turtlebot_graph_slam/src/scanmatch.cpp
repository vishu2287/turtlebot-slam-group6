#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "nav_msgs/OccupancyGrid.h"
using namespace Eigen;
//TODO FIX LASERSCAN SIZE so both scans have the same number of scans
sensor_msgs::PointCloud scanmatch(sensor_msgs::PointCloud one ,  sensor_msgs::PointCloud two){
 ROS_INFO_STREAM("Scanmatcher is starting...");
sensor_msgs::PointCloud test;
	//while ! converging  			//START of the icp iterations
	int secondsize = two.points.size();
	int saver[two.points.size()];
	
	double sum = 0;
	 // SEARCH CLOSEST NEIGHBOR
	 //-----------------------------------------------------------------------
	 for(int i = 0; i < two.points.size();i++){
		double prev = 1000000;
		for(int z = 0; z < one.points.size();z++){
			double length = sqrt(pow(one.points[z].x-two.points[i].x,2)+pow(one.points[z].y-two.points[i].y,2));
			if(length < prev){
				prev = length;
				//Save position of closest neighbor here
				saver[i] = z;
			}
		}
	 }

	// Calculate center points of both scans
	// ----------------------------------------------------
	 double centroidxone = 0;
	 double centroidyone = 0;
	 double centroidxtwo = 0;
	 double centroidytwo = 0;
	 double x = 0;
	 double y = 0;
	 for(int i = 0; i < two.points.size();i++){
		x+= two.points[i].x;
		y+= two.points[i].y;
	 }
	 centroidxtwo = x/two.points.size();			//Checken ob es double bleibt
	 centroidytwo = y/two.points.size();
	 x = 0;
	 y = 0;
	 for(int i = 0; i < one.points.size();i++){
		x+= one.points[i].x;
		y+= one.points[i].y;
	 }
	 centroidxone = x/one.points.size();
	 centroidyone = y/one.points.size();
	 ROS_INFO_STREAM("Correspondence and centroids created !");
	// Calculate Correlation matrix H
	// ----------------------------------------------------
	MatrixXd Mtd = MatrixXd::Zero(2,two.points.size());
	 for(int i = 0; i < two.points.size();i++){
		Mtd(0,i) = two.points[i].x-centroidxtwo;
		Mtd(1,i) = two.points[i].y-centroidytwo;
	 }
	 MatrixXd Mmd = MatrixXd::Zero(2,one.points.size());
	 for(int i = 0; i < one.points.size();i++){
		Mmd(0,i) = one.points[i].x-centroidxone;
		Mmd(1,i) = one.points[i].y-centroidyone;
	 }
	MatrixXd H = Mtd*Mmd.transpose() ;
	
	//std::cout << "MTD MMD = \n" << Mtd*Mmd.transpose() << std::endl;
	 ROS_INFO_STREAM("Matrizes created!");
	// Apply SVD (Singular value decomposition) TO GET rOTATION mATRIX r
	// ----------------------------------------------------
	 JacobiSVD<MatrixXd> svd(H);//SVD<MatrixXd> svdOfA(H);
	 MatrixXd U = svd.matrixU();
	 MatrixXd V = svd.matrixV();
	 MatrixXd R = V*U.transpose();
	// Estimate translation vector and build final transformation matrix
	// ----------------------------------------------------
	VectorXd centertwo = VectorXd::Zero(2,0);
	VectorXd centerone = VectorXd::Zero(2,0);
	VectorXd t = centertwo-R*centerone;
	Matrix3d T;
	T << R(0,0),R(0,1),t(0,0),
		R(1,0),R(1,1),t(1,0),
		0,0,1;

	// Final calculation
	// ----------------------------------------------------
	 for(int i = 0; i < one.points.size();i++){
		Vector3d M = VectorXd::Zero(3,0);
		M(0,0) = one.points[i].x;
		M(1,0) = one.points[i].y;
		M(2,0) = 1;
		Vector3d sol = T*M;
	 }


return test;
}
