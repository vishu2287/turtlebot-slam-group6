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

MatrixXd scanmatch(sensor_msgs::PointCloud one ,  sensor_msgs::PointCloud two){
MatrixXd robotpos = MatrixXd::Zero(3,1);
 ROS_INFO_STREAM("Scanmatcher is starting...");
	//while ! converging  			//START of the icp iterations
	int secondsize = two.points.size();
	int saver[two.points.size()];
	bool errors[two.points.size()];
	double delta = 0.0005;
	double distance = 1000;
	double oldDistance = 10000;
	double distances[two.points.size()];
	double robotx = 0;
	double roboty = 0;
	double robotphi = 0;
	double MAX_DISTANCE = 4;
	//for(int runthroughs = 0; runthroughs < Threshold ; runthroughs++){
		while(abs(oldDistance - distance) > delta) {
			oldDistance = distance;
			distance = 0;
		int numberofpairs = 0;
		 // SEARCH CLOSEST NEIGHBOR
		 //-----------------------------------------------------------------------
		 for(int i = 0; i < two.points.size();i++){
			double prev = 1000000;
			for(int z = 0; z < one.points.size();z++){
				double length = getDistance(one.points[z], two.points[i]);// sqrt(pow(one.points[z].x-two.points[i].x,2)+pow(one.points[z].y-two.points[i].y,2));
				if(length < prev){
					if(length >= MAX_DISTANCE){
						errors[i] = true;
						
					} else {
					//Save position of closest neighbor here
					saver[i] = z;
					distances[i] = length;	
					numberofpairs++;
					}
					prev = length;
				}
			}

			//std::cout << "The nearest neighbour of point " << i << " in the second scan is point " << saver[i] << " in the first scan"<< "\n";
		 }
	
		 for(int i = 0; i < two.points.size(); i++) {
			if(!errors[i])
		 	distance += distances[i];
		 }
		  std::cout << "Total distance = " << distance << "\n";

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
			if(!errors[i]){
			x+= two.points[i].x;
			y+= two.points[i].y;
			x1+= one.points[saver[i]].x;
			y1+= one.points[saver[i]].y;
			}
		 }
		 centroidxtwo = x/numberofpairs;			//Checken ob es double bleibt
		 centroidytwo = y/numberofpairs;
		 centroidxone = x1/numberofpairs;
		 centroidyone = y1/numberofpairs;
		 ROS_INFO_STREAM("Correspondence and centroids created !");
		// Calculate Correlation matrix H
		// ----------------------------------------------------
		MatrixXd Mtd = MatrixXd::Zero(2,numberofpairs);
		MatrixXd Mmd = MatrixXd::Zero(2,numberofpairs);
		int counter = 0;
		 for(int i = 0; i < two.points.size();i++){
			if(!errors[i]){
			Mtd(0,i-counter) = two.points[i].x-centroidxtwo;
			Mtd(1,i-counter) = two.points[i].y-centroidytwo;
			Mmd(0,i-counter) = one.points[saver[i]].x-centroidxone;
			Mmd(1,i-counter) = one.points[saver[i]].y-centroidyone;
			} else {
			counter++;
			}
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
		robotphi += acos(R(0,0));
	}

robotpos(0,0) = robotx;
robotpos(1,0) = roboty;
robotpos(2,0) = robotphi;
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
