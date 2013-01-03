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
sensor_msgs::PointCloud scanmatch(sensor_msgs::PointCloud one ,  sensor_msgs::PointCloud two){
 ROS_INFO_STREAM("Scanmatcher is starting...");
sensor_msgs::PointCloud test;
	int saver[two.points.size()]
	 // SEARCH CLOSEST NEIGHBOR
	 for(int i = 0; i < two.points.size();i++){
		
		for(int z = 0; z < one.points.size();z++){
			double length = sqrt(pow(one.points[z].x-two.points[i].x,2)+pow(one.points[z].y-two.points[i].y,2))
	
		}

	 }
return test;
}
