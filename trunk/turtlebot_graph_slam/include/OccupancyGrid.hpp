#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <Eigen/Dense>
#include "nav_msgs/OccupancyGrid.h"
#include <sensor_msgs/LaserScan.h>

nav_msgs::OccupancyGrid initializeOccupancyGridDefault();

nav_msgs::OccupancyGrid initializeOccupancyGrid(int SIZE, double resolution);

void publishOccupancyGrid(nav_msgs::OccupancyGrid og,ros::Publisher occupub);

nav_msgs::OccupancyGrid updateOccupancyGrid(nav_msgs::OccupancyGrid og, VectorXd mu, int t);