#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include <queue>
#include "nav_msgs/OccupancyGrid.h"



std::vector<std::vector<int> > frontierDetection(const nav_msgs::OccupancyGrid& grid, int x, int y);
bool isFrontier(const nav_msgs::OccupancyGrid& grid, int x, int y);
std::vector<int> getNeighbors(const nav_msgs::OccupancyGrid& grid, int x, int y);
int getMapValue(const nav_msgs::OccupancyGrid& grid, int x, int y);
std::vector<int> getSurrounding(const nav_msgs::OccupancyGrid& grid, int x, int y, int size);
