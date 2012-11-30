#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
double feature_extractor (const sensor_msgs::LaserScan::ConstPtr& msg,ros::Publisher publisher,ros::Publisher occupub);
