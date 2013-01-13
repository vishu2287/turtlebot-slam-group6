#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <exception>
#include <map>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "nav_msgs/OccupancyGrid.h"

using namespace Eigen;

sensor_msgs::PointCloud lasertransBase (const sensor_msgs::LaserScan::ConstPtr& msg){


    sensor_msgs::PointCloud cloud;

    int num_ranges = msg->ranges.size();
    cloud.points.resize(num_ranges);

    float incr = msg->angle_increment;
    float minAngle = msg->angle_min;
    double angle;
    for (int j = 0; j < num_ranges; j++){
        angle = (double)(j*incr)+minAngle;
        double x = msg->ranges[j] * cos(angle);
        double y = msg->ranges[j] * sin(angle);
        geometry_msgs::Point32 p;
        p.x = x;
        p.y = y;
        cloud.points[j] = p;
    }
    return cloud;
}
sensor_msgs::PointCloud lasertrans (const sensor_msgs::LaserScan::ConstPtr& msg){


    tf::TransformListener tfListener_;
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector_;

    try
    {
        projector_.projectLaser(*msg,cloud);   //Transform laser cloud to world frame

    }
    catch (...)
    {
        sensor_msgs::PointCloud error;
        return error;
        // std::cout << e.what();                       //TODO Fix errors, they still occur decomment to see whats happening
    }
    return cloud;
}

