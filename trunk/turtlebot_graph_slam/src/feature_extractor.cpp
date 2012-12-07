#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "nav_msgs/OccupancyGrid.h"
using namespace Eigen;
//https://gt-ros-pkg.googlecode.com/svn-history/r2368/trunk/hrl/simple_occupancy_grid/src/occupancy_grid.cpp
//http://cs225turtle.googlecode.com/svn/trunk/project2/local_obstacles/src/local_obstacles.cpp
int laserscancount = 0;
double PI = 3.14;
const static double MIN_SCAN_ANGLE_RAD = -15.0 / 180 * M_PI; //@TODO: Set the range
const static double MAX_SCAN_ANGLE_RAD = +15.0 / 180 * M_PI; //@TODO: Set the range
const static float PROXIMITY_RANGE_M = 1; // Should be smaller than sensor_msgs::LaserScan::range_max
MatrixXd feature_extractor(const sensor_msgs::LaserScan::ConstPtr& msg,
		ros::Publisher publisher, ros::Publisher occupub) {

	/*		Populate a Occupancy Grid
	 --------------------------------------------------------------------------------------*/
	/*int SIZE = 2000; //Size of the occupancygrid

	 nav_msgs::OccupancyGrid og;
	 og.info.resolution = 0.5;
	 og.header.frame_id = "/world";
	 og.info.origin.position.x = -SIZE/2;
	 og.info.origin.position.y = -SIZE/2;
	 og.header.stamp = ros::Time::now();
	 og.info.width = SIZE;
	 og.info.height = SIZE;
	 og.data.resize(SIZE * SIZE);*/

	/*		LASER SCANS
	 -------------------------------------------------------------------------------------------*/
	unsigned int minIndex = ceil(
			(MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
	unsigned int maxIndex = ceil(
			(MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
	float closestRange = msg->ranges[minIndex];
	std::vector<int> laserpose;
	int lasercount = maxIndex - minIndex;

	MatrixXd Z = MatrixXd::Zero(3, lasercount);
	double rad = MIN_SCAN_ANGLE_RAD;

	for (unsigned int currIndex = minIndex; currIndex < maxIndex;
			currIndex++) {
		double help = rad;
		if(rad < 0){
			rad = 2*PI - rad;
		}
		Z(0, currIndex - minIndex) = msg->ranges[currIndex];
		Z(1, currIndex - minIndex) = rad;
		Z(2, currIndex - minIndex) = 1;
		if (msg->ranges[currIndex] < msg->range_max) {
			laserpose.push_back(currIndex); //Get obstacle positions
		}
		
	        std::cout << "Z = \n" << rad<< std::endl;
		rad = help;
		rad += msg->angle_increment;
	}

	/*			START CREATING CLOUD
	 ----------------------------------------------------------------------*/
	/* tf::TransformListener tfListener_;
	 tfListener_.setExtrapolationLimit(ros::Duration(0.1));
	 sensor_msgs::PointCloud cloud;
	 laser_geometry::LaserProjection projector_;
	 try
	 {
	 projector_.transformLaserScanToPointCloud("/base_link", *msg, cloud, tfListener_);	//Transform laser cloud to world frame

	 }
	 catch (tf::TransformException& e)
	 {
	 //std::cout << e.what();			//TODO Fix errors, they still occur decomment to see whats happening
	 }*/

//grid_x = (unsigned int)((cloud.points[i].x - og.info.origin.position.x) / og.info.resolution);
//grid_y = (unsigned int)((cloud.points[i].y - map.info.origin.position.y) / map.info.resolution);

	/*			UPDATE OCCUPANCY GRID
	 ----------------------------------------------------------------------*/
	//PUBLISH OCCUPANCY GRID AND POINT CLOUD
	//for(int i = 0; i<og.data.size();i++){
	//	og.data[i] = -1;
	//}
	/* for(int i = 0; i < cloud.points.size();i++){
	 /// og.info.resolution
	 int grid_x = (unsigned int)((cloud.points[i].x - og.info.origin.position.x)/og.info.resolution);
	 int grid_y = (unsigned int)((cloud.points[i].y - og.info.origin.position.y)/og.info.resolution);
	 // ROS_INFO_STREAM("Robot moved to position x:"<< (grid_x*2000)+grid_y);
	 og.data[(grid_x*2000)+grid_y] = 100;		//TODO somehow turn it
	 //ROS_INFO_STREAM("Robot moved to position x:"<< grid_y);

	 }
	 publisher.publish(cloud);
	 occupub.publish(og);*/
	return Z;
}
