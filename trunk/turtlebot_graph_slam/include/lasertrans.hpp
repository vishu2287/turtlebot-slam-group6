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


//Table 11.4 Page 349
//@TODO: Implement
sensor_msgs::PointCloud lasertrans (const sensor_msgs::LaserScan::ConstPtr& msg);


	
