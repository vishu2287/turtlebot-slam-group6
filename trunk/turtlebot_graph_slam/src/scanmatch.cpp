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
	 double x1 = 0;
	 double y1 = 0;
	 for(int i = 0; i < two.points.size();i++){
		x+= two.points[i].x;
		y+= two.points[i].y;
		x1+= one.points[saver[i]].x;
		y1+= one.points[saver[i]].y;
	 }
	 centroidxtwo = x/two.points.size();			//Checken ob es double bleibt
	 centroidytwo = y/two.points.size();
	 centroidxone = x1/two.points.size();
	 centroidyone = y1/two.points.size();
	 ROS_INFO_STREAM("Correspondence and centroids created !");
	// Calculate Correlation matrix H
	// ----------------------------------------------------
	MatrixXd Mtd = MatrixXd::Zero(2,two.points.size());
	MatrixXd Mmd = MatrixXd::Zero(2,two.points.size());
	MatrixXd onebub = MatrixXd::Ones(2,5);
	MatrixXd twobub = MatrixXd::Ones(2,5);
	 for(int i = 0; i < two.points.size();i++){
		Mtd(0,i) = two.points[i].x-centroidxtwo;
		Mtd(1,i) = two.points[i].y-centroidytwo;
		Mmd(0,i) = one.points[saver[i]].x-centroidxone;
		Mmd(1,i) = one.points[saver[i]].y-centroidyone;
	 }
	MatrixXd H = Mmd*Mtd.transpose() ;
	ROS_INFO_STREAM("Matrizes created!");
	// Apply SVD (Singular value decomposition) TO GET rOTATION mATRIX r
	// ----------------------------------------------------
	 JacobiSVD<MatrixXd> svd(H,Eigen::ComputeThinU | Eigen::ComputeThinV);//(H);//SVD<MatrixXd> svdOfA(H);
	 MatrixXd U = svd.matrixU();
	 MatrixXd V = svd.matrixV();
	 MatrixXd R = V*U.transpose();
		std::cout << "R" << R;
	// Estimate translation vector and build final transformation matrix
	// ----------------------------------------------------
	std::cout << "R" << R;
	VectorXd centertwo(2);
	VectorXd centerone(2);
	centertwo(0) = centroidxtwo;
	centertwo(1) = centroidytwo;
	centerone(0) = centroidxone;
	centerone(1) = centroidyone;
	VectorXd t = centertwo-R*centerone;
	MatrixXd Tmat =MatrixXd::Zero(3,3);
	Tmat << R(0,0),R(0,1),t(0),
		R(1,0),R(1,1),t(1),
		0,0,1;
	// Final calculation
	// ----------------------------------------------------
	 for(int i = 0; i < one.points.size();i++){
		Vector3d M(3);
		M(0) = one.points[i].x;
		M(1) = one.points[i].y;
		M(2) = 1;
		Vector3d sol = Tmat*M;
		one.points[saver[i]].x = sol(0);
	        one.points[saver[i]].y = sol(1);
	 }
return one;
}
