#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <Eigen/Dense>
#include "nav_msgs/OccupancyGrid.h"

nav_msgs::OccupancyGrid initializeOccupancyGridDefault();

nav_msgs::OccupancyGrid initializeOccupancyGrid(int SIZE, double resolution);

int calculateFakeSize(VectorXd mu);

nav_msgs::OccupancyGrid updateOccupancyGrid(nav_msgs::OccupancyGrid og, VectorXd mu, int numberOfPoses, int erase);
