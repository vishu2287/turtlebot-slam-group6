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
  int laserscancount = 0;
//laser_geometry::LaserProjection projector;
//tf::TransformListener listener;
  const static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
  const static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
  const static float PROXIMITY_RANGE_M = 1; // Should be smaller than sensor_msgs::LaserScan::range_max
double feature_extractor (const sensor_msgs::LaserScan::ConstPtr& msg,ros::Publisher publisher){

      unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
      unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
      float closestRange = msg->ranges[minIndex];
      for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
        if (msg->ranges[currIndex] < closestRange) {
          closestRange = msg->ranges[currIndex];   }
      }
      //ROS_INFO_STREAM("Range: " << closestRange);
	 //ROS_INFO_STREAM("number of laserscan " << laserscancount++);
     	// ROS_INFO_STREAM("Number of Laserscandata: " << maxIndex-minIndex);
/*			START CREATING CLOUD 
----------------------------------------------------------------------*/
    tf::TransformListener tfListener_;
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
    sensor_msgs::PointCloud cloud;
     laser_geometry::LaserProjection projector_;
    try
    {
            projector_.transformLaserScanToPointCloud("/base_laser_link", *msg, cloud, tfListener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
	return 0;
    }
	for(int i = 0 ; i < cloud.points.size();i+=3){
	 ROS_INFO_STREAM("cloud points: " << cloud.points[i]);
	 ROS_INFO_STREAM("cloud points: " << cloud.points[i+1]);
         ROS_INFO_STREAM("cloud points: " << cloud.points[i+2]);
	}

    publisher.publish(cloud);
  //sensor_msgs::PointCloud cloud;
  //projector.transformLaserScanToPointCloud("base_link",*msg, cloud,listener);
  //ROS_INFO_STREAM("Number of Laserscandata: " << sizeof(cloud.points));
return 0;
}
