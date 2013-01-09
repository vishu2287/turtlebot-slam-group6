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
    ROS_INFO_STREAM("MAP HEIGHT : " << og.info.height);
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
//        for(int i = 0; i < og.data.size();i++){
//		og.data[i] = -1;
//	}
//	for(int i = 0 ; i < laserscansaver.size() ; i++){
//        for(int i = laserscansaver.size() ; i < laserscansaver.size() ; i++){
            int i = laserscansaver.size()-1;
		sensor_msgs::PointCloud temp = lasertrans(laserscansaver[i]);
//		if(i>= poses.size())
//			break;
        double xPose = poses[i](0,0);
        double yPose = poses[i](1,0);
        double yaw = poses[i](2,0);

		for(int z = 0; z < temp.points.size() ; z++){

            // Equation to solve: A*(x-p)+p
            // In order to have the points of the measurement rotated the same way the robot did

            // Initialised position of the points on the grid, but without rotation
            int grid_x = (int)((temp.points[z].x+xPose)/RESOLUTION + SIZE/2);
            int grid_y = (int)((temp.points[z].y+yPose)/RESOLUTION + SIZE/2);

            // Now apply the equation mentioned above to rotate around the robot
            double a11 =  cos(yaw);
            double a12 = -sin(yaw);
            double a21 =  sin(yaw);
            double a22 =  cos(yaw);

            double x1 = grid_x;
            double x2 = grid_y;

            double p1 = xPose/RESOLUTION + SIZE/2;
            double p2 = yPose/RESOLUTION + SIZE/2;

            double diffX = x1-p1;
            double diffY = x2-p2;

            grid_x = a11*diffX + a12*diffY + p1;
            grid_y = a21*diffX + a22*diffY + p2;

            og.data[((grid_y*og.info.width)+grid_x)] = 100;
        }

//	}
    return og;
}
