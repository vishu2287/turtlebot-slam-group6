#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <Eigen/Dense>
#include <lasertrans.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
using namespace Eigen;
double RESOLUTION;
int SIZE;
nav_msgs::OccupancyGrid initializeOccupancyGrid(int length, double resolution) {
    RESOLUTION = resolution;
    SIZE  = length;
    nav_msgs::OccupancyGrid og;
    og.info.resolution = resolution;
    og.header.frame_id = "/world";
    og.info.origin.position.x = -SIZE/2*resolution;
    og.info.origin.position.y = -SIZE/2*resolution;
    og.header.stamp = ros::Time::now();
    og.info.width = SIZE;
    og.info.height = SIZE;
    og.data.resize(SIZE * SIZE);

    return og;
}
//publish Occupacy grid
void publishOccupancyGrid(nav_msgs::OccupancyGrid og,ros::Publisher occupub){
	occupub.publish(og);
}
    /*		Populate a Occupancy GridFF
    --------------------------------------------------------------------------------------*/
nav_msgs::OccupancyGrid updateOccupancyGrid(nav_msgs::OccupancyGrid og, std::vector < sensor_msgs::LaserScan::ConstPtr > laserscansaver, std::vector<MatrixXd> poses){
    // ROS_INFO_STREAM("Occupancy Grid has height: "<<og.info.height<<" and width: "<<og.info.width);
        for(int i = 0; i < og.data.size();i++){
		og.data[i] = -1;
	}
	double xtranspose = 0;
	double ytranspose = 0;
	for(int i = 0 ; i < laserscansaver.size() ; i++){
		sensor_msgs::PointCloud temp = lasertrans(laserscansaver[i]);
		if(i>= poses.size())
			break;
	xtranspose+=poses[i](0,0);
	ytranspose+=poses[i](1,0);
	//ROS_INFO_STREAM("Occupancy Grid has height: "<<xtranspose<<" and width: "<<ytranspose);
		for(int z = 0; z < temp.points.size() ; z++){
			        int grid_x = (unsigned int)(((temp.points[z].x+xtranspose)/RESOLUTION + SIZE/2));
   			        int grid_y = (unsigned int)(((temp.points[z].y+ytranspose)/RESOLUTION + SIZE/2));
       			        og.data[((grid_y*og.info.width)+grid_x)] = 100;
		}
	}
    /*for(int pose = (t+1)*3; pose < mu.size(); pose += 3) {
        double x = mu(pose);
        if(x < -10 || x > 10){ //There are sometimes values of 30000000 and - 30000000 ... for testing
            x = 0;
        }
        double y = mu(pose+1);
        if(y < -10 || y > 10){ //There are sometimes values of 30000000 and - 30000000 ... Otherwise the occupancy grid WONT work
            y = 0;
        }
        double z = mu(pose+2);
        int grid_x = (unsigned int)((x/RESOLUTION + SIZE/2));
        int grid_y = (unsigned int)((y/RESOLUTION + SIZE/2));
        //ROS_INFO_STREAM("Grid X = "<< grid_x << ", Grid Y =" << grid_y);
        og.data[((grid_y*og.info.width)+grid_x)] = 100;
    }*/
    return og;
}
