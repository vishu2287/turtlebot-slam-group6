#include "ros/ros.h"
#include <Eigen/Dense>
#include <feature_extractor.hpp>
#include <graph_slam.hpp>
#include <OccupancyGrid.hpp>
#include <GraphBasedAlgo1.hpp>
#include <Constraint.hpp>
#include <robotpos.hpp>
#include <simple_slam.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <scanmatch.hpp>
#include <lasertrans.hpp>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>
using namespace std;
using namespace Eigen;

/**
  Publishers
*/
ros::Publisher point_cloud_publisher_;
ros::Publisher point_cloud_publisher_second;
ros::Publisher point_cloud_publisher_combined;
ros::Publisher occupub;
ros::Publisher robotPosePublisher;
ros::Publisher marker_pub;
robotpospub robopub;

// Model for the world and the path
nav_msgs::OccupancyGrid occupancyGrid;
geometry_msgs::PoseArray posearray;

// Currently saved laser scan
sensor_msgs::LaserScan::ConstPtr currentScan;

// Scans for every node
std::vector < sensor_msgs::LaserScan::ConstPtr > scans;

// Lists for the Graph
std::vector<Vector3d> nodes;
std::vector<Constraint> constraints;

// PointClouds
sensor_msgs::PointCloud prevPointCloud;
sensor_msgs::PointCloud currentPointCloud;

// Help variables for the velocity callback function
double prevX = 0;
double prevY = 0;
double prevZ = 0;
double veryLastX;
double veryLastY;
double veryLastYaw;
bool initialised;
bool loopDetected;

// Distance after which a laser scan should be matched again
double MATCH_DISTANCE = 0.3;
void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    /**
      Simply save the current scan.
      It will be used by the velocity callback function as seen fit.
    */
    currentScan = msg;
}

void publishPath(){
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id =  "/world";
    points.header.stamp = line_strip.header.stamp =  ros::Time::now();
    points.ns = line_strip.ns =  "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w =  1.0;
    points.id = 0;
    line_strip.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_LIST;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    posearray.header.stamp = ros::Time::now();
    posearray.header.frame_id = "/world";
    posearray.poses.resize(nodes.size());
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.02;
    for(unsigned int i = 0; i < posearray.poses.size(); i++){
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(nodes[i](2));
        posearray.poses[i].position.x = nodes[i](0);
        posearray.poses[i].position.y = nodes[i](1);
        posearray.poses[i].position.z = 0;
        posearray.poses[i].orientation.x = odom_quat.x;
        posearray.poses[i].orientation.y = odom_quat.y;
        posearray.poses[i].orientation.z = odom_quat.z;
        posearray.poses[i].orientation.w =  odom_quat.w;
        geometry_msgs::Point p;
        p.x = nodes[i](0);
        p.y = nodes[i](1);
        points.points.push_back(p);
    }

    // Display lines according to the constraints
    for(int c = 0; c<constraints.size(); c++)
    {
        geometry_msgs::Point p1;
        p1.x = nodes[constraints[c].i](0);
        p1.y = nodes[constraints[c].i](1);
        geometry_msgs::Point p2;
        p2.x = nodes[constraints[c].j](0);
        p2.y = nodes[constraints[c].j](1);
        line_strip.points.push_back(p1);
        line_strip.points.push_back(p2);
    }
    robotPosePublisher.publish(posearray);
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
}

double distance(double x1,double x2, double y1, double y2){
    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

bool rotation(double rot1, double rot2) {
    if(rot1 < 0 && rot2 > 0) {
        rot1 *= -1;
        rot2 = rot1 + M_PI/2;
    }
    if(rot2 < 0 && rot1 > 0) {
        rot2 *= -1;
        rot2 = rot2 + M_PI/2;
    }
    double rotDifference = rot2-rot1;
    if(rotDifference >= 0.3) {
        return true;
    }
    return false;
}

sensor_msgs::PointCloud transformCloud(sensor_msgs::PointCloud cloud, double t1, double t2, double angle)
{
    double a11 =  cos(-angle);
    double a12 = -sin(-angle);
    double a21 =  sin(-angle);
    double a22 =  cos(-angle);

    for(int i = 0; i<cloud.points.size(); i++)
    {
        double x1 = cloud.points[i].x;
        double x2 = cloud.points[i].y;

        double diffX = x1-t1;
        double diffY = x2-t2;

        cloud.points[i].x = a11*diffX + a12*diffY;
        cloud.points[i].y = a21*diffX + a22*diffY;
    }
    return cloud;
}

void vel_callback(const nav_msgs::Odometry& msg) {

    // Get the current position of the robot
    double newX = msg.pose.pose.position.x;
    double newY = msg.pose.pose.position.y;
    geometry_msgs::Quaternion odom_quat = msg.pose.pose.orientation;
    double newZ = tf::getYaw(odom_quat);


    // If this is the first call create the first node
    if(!initialised && currentScan){

        // Save the current position as previous
        prevX = newX;
        prevY = newY;
        prevZ = newZ;

        //Save first position of the robot
        Vector3d firstNode;
        firstNode(0) = newX;
        firstNode(1) = newY;
        firstNode(2) = newZ;
        nodes.push_back(firstNode);

        //Publish first pose in pose array
        publishPath();

        // Save the current scan
        scans.push_back(currentScan);

        initialised = true;
        occupancyGrid = updateOccupancyGrid(occupancyGrid,scans,nodes);
        publishOccupancyGrid(occupancyGrid,occupub);
        // Initialize robotpublisher He will make a ros::spin at this point so the whole program will be runthrough again
        robopub.robotpos(0,0,0,0,0);
    }

    // If the robot has walked far enough to make the second scan
    if(distance(prevX,newX,prevY,newY)>= MATCH_DISTANCE) {
        std::cout << "Nodes: " << nodes.size() << std::endl;
        std::cout << "Constraints: " << constraints.size() << std::endl;

        // Create new node for current robot pose
        Vector3d newNode;
        newNode(0) = newX;
        newNode(1) = newY;
        newNode(2) = newZ;

        // Add new node to the graph
        nodes.push_back(newNode);

        // Label node with current scan
        scans.push_back(currentScan);

        // Create constraints between current node and previous node
        // and current node and close nodes (loop closing)
        int j = nodes.size()-1;
        loopDetected = false;
        for(int i = 0; i<j; i++)
        {
            double oldX = nodes[i](0);
            double oldY = nodes[i](1);
            double oldZ = nodes[i](2);
            double nodeDistance = distance(oldX,newX,oldY,newY);
            if(nodeDistance < MATCH_DISTANCE || i == j-1)
            {
                // Mark as loop detection
                if(nodeDistance < MATCH_DISTANCE)
                    loopDetected = true;

                // Transform the ith and jth scan to PointClouds
                prevPointCloud = lasertransBase(scans[i]);
                currentPointCloud = lasertransBase(scans[j]);

                // Calculate odometry between nodes
                double diffX = newX-oldX;
                double diffY = newY-oldY;
                double xX = cos(-oldZ)*diffX - sin(-oldZ)*diffY;
                double xY = sin(-oldZ)*diffX + cos(-oldZ)*diffY;
                double xYaw = newZ-oldZ;

                // Transform the previous scan according to odometry to have an estimate for scanmatch
                prevPointCloud = transformCloud(prevPointCloud, xX, xY, xYaw);

                // Use scanmatch for both clouds
                Vector3d correction = scanmatch(prevPointCloud, currentPointCloud);

                // In order to test Graph SLAM with the actual odometry, with random noise
                Vector3d zTransformation;
                zTransformation(0) = xX+correction(0);
                zTransformation(1) = xY+correction(1);
                zTransformation(2) = xYaw+correction(2);

                // Create a constraint between ith and jth node and save measurement transform
                Constraint c;
                c.i = i;
                c.j = j;
                c.z = zTransformation;
                double uncertainty = 100;
                Matrix3d covariance = Matrix3d::Identity(3, 3);
                covariance(0,0) = uncertainty;
                covariance(1,1) = uncertainty;
                covariance(2,2) = uncertainty;
                c.omega = covariance.inverse();

                // Add constraint to the graph
                constraints.push_back(c);
            }
        }

        // Perform Graph SLAM optimization
        nodes = algorithm1(nodes, constraints);

        // Set the robot pose to the optimized pose
        robopub.robotxx = nodes[nodes.size()-1](0);
        robopub.robotyy = nodes[nodes.size()-1](1);
        robopub.robotyyaw = nodes[nodes.size()-1](2);

        //Publish Pose Array
        publishPath();

        // Publish clouds
        point_cloud_publisher_.publish(prevPointCloud);
        point_cloud_publisher_second.publish(currentPointCloud);

        // Update grid
        occupancyGrid = updateOccupancyGrid(occupancyGrid,scans,nodes);
        publishOccupancyGrid(occupancyGrid,occupub);

        // Save current pose as previous
        prevX = newX;
        prevY = newY;
        prevZ = newZ;
    }

    // Update robotpublisher values, only approximated odometry
    robopub.robotxx += newX-veryLastX;
    robopub.robotyy += newY-veryLastY;
    robopub.robotyyaw += newZ-veryLastYaw;

    // For approximating the odometry
    veryLastX = newX;
    veryLastY = newY;
    veryLastYaw = newZ;
}

/**
  Whenever a loop closing is detected, the whole occupancy grid is corrected.
  This is an expensive procedure, so it is seperated here.
  */
bool blocked;
void timer_callback(const ros::TimerEvent&) {
    if(!blocked && loopDetected)
    {
        blocked = true;
        occupancyGrid = updateOccupancyGridAll(occupancyGrid,scans,nodes);
        blocked = false;
        loopDetected = false;
    }
    publishOccupancyGrid(occupancyGrid,occupub);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    occupancyGrid = initializeOccupancyGrid(2000, 0.05);
    ros::Timer timer = n.createTimer(ros::Duration(5), timer_callback);
    ros::Subscriber laserSub = n.subscribe("base_scan", 100, callback);
    ros::Subscriber velSub = n.subscribe("odom", 1, vel_callback);
    robotPosePublisher = n.advertise<geometry_msgs::PoseArray> ("/poses", 100, false);
    point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud> ("/cloud", 100,false);
    point_cloud_publisher_second = n.advertise<sensor_msgs::PointCloud> ("/cloud2", 100,false);
    point_cloud_publisher_combined = n.advertise<sensor_msgs::PointCloud> ("/cloudcombined", 100,false);
    marker_pub = n.advertise<visualization_msgs::Marker>("/lines", false);
    occupub = n.advertise<nav_msgs::OccupancyGrid> ("/world", 100,false);

    ros::spin();

    return 0;
}

