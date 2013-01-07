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
#include <geometry_msgs/Point32.h>
using namespace Eigen;
//TODO FIX LASERSCAN SIZE so both scans have the same number of scans

double getDistance(geometry_msgs::Point32 one, geometry_msgs::Point32 two) {
	return sqrt(pow((one.x - two.x), 2) + pow((one.y - two.y), 2));
}

MatrixXd scanmatch(sensor_msgs::PointCloud one ,  sensor_msgs::PointCloud two,double robotx, double roboty, double robotphi){
MatrixXd robotpos = MatrixXd::Zero(1,3);
 ROS_INFO_STREAM("Scanmatcher is starting...");
	//while ! converging  			//START of the icp iterations
	int secondsize = two.points.size();
	int saver[two.points.size()];
	double delta = 0.01;
	double distance = 1000;
	double oldDistance = 10000;
	double distances[two.points.size()];
	//for(int runthroughs = 0; runthroughs < Threshold ; runthroughs++){
		while(abs(oldDistance - distance) > delta) {
			oldDistance = distance;
			distance = 0;
		 // SEARCH CLOSEST NEIGHBOR
		 //-----------------------------------------------------------------------
		 for(int i = 0; i < two.points.size();i++){
			double prev = 1000000;
			for(int z = 0; z < one.points.size();z++){
				double length = getDistance(one.points[z], two.points[i]);// sqrt(pow(one.points[z].x-two.points[i].x,2)+pow(one.points[z].y-two.points[i].y,2));
				if(length < prev){
					prev = length;
					//Save position of closest neighbor here
					saver[i] = z;
					distances[i] = length;				}
			}

			//std::cout << "The nearest neighbour of point " << i << " in the second scan is point " << saver[i] << " in the first scan"<< "\n";
		 }
		 for(int i = 0; i < two.points.size(); i++) {
		 	distance += distances[i];
		 }
		 // std::cout << "Total distance = " << distance << "\n";

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
			//std::cout << "R" << R;
		// Estimate translation vector and build final transformation matrix
		// ----------------------------------------------------
		//std::cout << "R" << R;
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
		//std::cout << "Translation Matrix" << R;
		// Final calculation
		// ----------------------------------------------------
		 for(int i = 0; i < one.points.size();i++){
			Vector3d M(3);
			M(0) = one.points[i].x;
			M(1) = one.points[i].y;
			M(2) = 1;
			VectorXd sol = Tmat*M;
			one.points[i].x = sol(0);
			one.points[i].y = sol(1);
		 }
		Vector3d temp_coord(3);
		temp_coord(0) = robotx;
		temp_coord(1) = roboty;
		temp_coord(2) = 1;
		VectorXd robotsol = Tmat*temp_coord;
		robotx = robotsol(0);
		roboty = robotsol(1);
	}
std::cout << "ROBOTX" << -robotx;
std::cout << "ROBOTY" << -roboty;
robotpos(0,0) = -robotx;
robotpos(0,1) = -roboty;
return robotpos;
}
/*bool contains(int test [], int i){
	for(int i= 0; i < sizeof(test) ; i++){
		if(test[i] == i)
			return true;
	}
	return false;
}
sensor_msgs::PointCloud scanmatch(sensor_msgs::PointCloud one ,  sensor_msgs::PointCloud two){
 ROS_INFO_STREAM("Scanmatcher is starting...");
	//while ! converging  			//START of the icp iterations
	int secondsize = two.points.size();
	int saver[two.points.size()];
	double lengthsaver[two.points.size()];
	int Threshold = 5;
	std::vector <geometry_msgs::Point32 > firstpoints;
	std::vector <geometry_msgs::Point32 > secondpoints;
	for(int runthroughs = 0; runthroughs < Threshold ; runthroughs++){
		double sum = 0;
		 // SEARCH CLOSEST NEIGHBOR
		 //-----------------------------------------------------------------------
		 for(int i = 0; i < two.points.size();i++){
			double prev = 1000000;
			for(int z = 0; z < one.points.size();z++){
				double length = sqrt(pow(one.points[z].x-two.points[i].x,2)+pow(one.points[z].y-two.points[i].y,2));
				if(length == 0)
					continue;
				if(length < prev){
					//First occurence of point pair
					if(lengthsaver[i] == 0){
						 //Save position of closest neighbor here
						 lengthsaver[i] = length;
						 saver[i] = z;
					}
					//If pair already exists
					else if(contains(saver,z)){
						if(lengthsaver[i] < length){
						}
						// If save length > current length
						else{
							lengthsaver[i] = length;
					                saver[i] = z;
						}
					} else {
						lengthsaver[i] = length;
						saver[i] = z;
					}
					prev = length;
				}
			}
		 }
		for(int i = 0; i < two.points.size();i++){
			
		}*/
