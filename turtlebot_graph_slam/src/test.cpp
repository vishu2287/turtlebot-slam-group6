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

using namespace Eigen;
	ros::Publisher point_cloud_publisher_;
	ros::Publisher point_cloud_publisher_second;
	ros::Publisher point_cloud_publisher_combined;
	ros::Publisher occupub;
	ros::Publisher robotPathPublisher;
	ros::Publisher robotPosePublisher;
	ros::Publisher marker_pub;
//std::vector <Vector2d> odoms; //Odometry saver TODO somehow did not work on 1 system
std::vector <MatrixXd> Zs; //Z saver TODO Test if this works!
int t = 0;
int deltaT = 1;
double prevX = 0;
double prevY = 0;
double prevZ = 0;
nav_msgs::OccupancyGrid world;
nav_msgs::Path robotPath;
robotpospub robopub;
geometry_msgs::PoseArray posearray;
//Robotvariables
double robotx = 0;
double roboty = 0;
//MatrixXd mut = MatrixXd::Zero(3, 1);
MatrixXd u = MatrixXd::Zero(2, 1);

bool flag = false;
double speed;
double angular;
//MAINTAIN a list of laserscans for each pose and of all robotposes
std::vector < sensor_msgs::LaserScan::ConstPtr > laserscansaver;
std::vector<Vector3d> nodes;
// maintain two laserscans if robot moves 0.5 meters forward
sensor_msgs::LaserScan::ConstPtr currentScan;
sensor_msgs::LaserScan::ConstPtr prevScan;
// 3 pointclouds for two different laserscans and the ICP output cloud
sensor_msgs::PointCloud pointCloud1;
sensor_msgs::PointCloud pointCloud2;
sensor_msgs::PointCloud combined;
// TUNABLE PARAMETERS
double MATCH_DISTANCE = 0.5;	// Distance after which a laser scan should be matched again

// Constraint list
std::vector<Constraint> constraints;

/*	Laserscancallback needed for feature extraction
--------------------------------------------------------------------------------------*/
void callback(const sensor_msgs::LaserScan::ConstPtr& msg) { // Always call graph slam for new laser readings
    /**
      Simply save the current scan.
      It will be used by the velocity callback function as seen fit.
    */
    currentScan = msg;
}
/*	Process X vector to Pose Array and Connect each node with lines
--------------------------------------------------------------------------------------*/
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
/*	Robot Position function, values from Graphslam should be incorporated here
--------------------------------------------------------------------------------------*/
void rob_callback(const ros::TimerEvent&) {
	//current robot position in world is always 0,0

	/*std::vector<int> is;
	is.push_back(0);
	is.push_back(1);
	is.push_back(2);
	is.push_back(3);
	is.push_back(4);

	std::vector<int> js;
	js.push_back(1);
	js.push_back(2);
	js.push_back(3);
	js.push_back(4);
	js.push_back(5);

	VectorXd x(18, 1);
	x << 	  0,   0,   0.3,
			  3,   1,   0.3,
			  5,   3,   1.5,
			  4,   5,   1.9,
			  4,   8,   0.6,
			  7,  10,   0.4;

	std::vector<Vector3d> z;
	Vector3d z1(3,1);
//	z1 <<	3.5, 0.5,  0.3;
//	Vector3d z2(3,1);
//	z2 <<	1.5, 1.5,  0.3;
//	Vector3d z3(3,1);
//	z3 <<	0.5, 2.5,  0.3;
//	Vector3d z4(3,1);
//	z4 <<	0.5, 2.5,   -1;
//	Vector3d z5(3,1);
//	z5 <<	  2, 1.5, -0.3;

	z1 <<	3.53,   0,  0.3;
	Vector3d z2(3,1);
	z2 <<	2.12,   0,  0.3;
	Vector3d z3(3,1);
	z3 <<	2.55,   0,  0.3;
	Vector3d z4(3,1);
	z4 <<	2.55,   0, -0.3;
	Vector3d z5(3,1);
	z5 <<	2.50,   0, -0.3;

	z.push_back(z1);
	z.push_back(z2);
	z.push_back(z3);
	z.push_back(z4);
	z.push_back(z5);

	Matrix3d covariance = Matrix3d::Identity(3, 3);
//	covariance(0,0) = 50;
//	covariance(1,1) = 60;
//	covariance(2,2) = 70;
	std::vector<Matrix3d> omegas;
	omegas.push_back(covariance.inverse());
	omegas.push_back(covariance.inverse());
	omegas.push_back(covariance.inverse());
	omegas.push_back(covariance.inverse());
	omegas.push_back(covariance.inverse());

	algorithm1(x, z, omegas, is, js);*/
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

/*		Velocity callback function, called continuously
--------------------------------------------------------------------------------------*/
bool initialised;
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
        laserscansaver.push_back(currentScan);

        // Save the first scan as previous
        prevScan = currentScan;

        initialised = true;
	
        std::cout << "Origin node created" << std::endl;
        std::cout << "Number of saved poses = " << nodes.size() << std::endl;
        std::cout << "Number of saved scans = " << laserscansaver.size() << std::endl;
        std::cout << "-----" << std::endl;

        // Initialize robotpublisher He will make a ros::spin at this point so the whole program will be runthrough again
        robopub.robotpos(0,0,0,0,0);
    }

    // If the robot has walked far enough to make the second scan
    if(distance(prevX,newX,prevY,newY)>= MATCH_DISTANCE /*|| rotation(prevZ, newZ)*/) {

        // Save current scan
        laserscansaver.push_back(currentScan);

        // Transform the current and last scan to PointClouds
        pointCloud1 = lasertransBase(currentScan);
        pointCloud2 = lasertransBase(prevScan);

        // If both clouds are not empty
        if(!pointCloud1.points.empty() || !pointCloud2.points.empty()){

            // Create new node for current robot pose
            Vector3d newNode;
            newNode(0) = newX;
            newNode(1) = newY;
            newNode(2) = newZ;

            // Add new node to the graph
            nodes.push_back(newNode);

            // Create constraints between current node and previous node
            // and current node and close nodes (loop closing)
            bool loopDetected = false;
            int j = nodes.size()-1;
            for(int i = 0; i<j; i++)
            {
                double oldX = nodes[i](0);
                double oldY = nodes[i](1);
                double nodeDistance = distance(oldX,newX,oldY,newY);
                if(nodeDistance < MATCH_DISTANCE || i == j-1)
                {
                    // Mark as loop detection
                    if(nodeDistance < MATCH_DISTANCE)
                        loopDetected = true;

                    // Use scanmatch for both clouds
                    Vector3d zTransformation = scanmatch(pointCloud1,pointCloud2);

                    // Get X, Y and yaw transformation according to scanmatcher
                    double zX = zTransformation(0,0);
                    double zY = zTransformation(1,0);
                    double zYaw = zTransformation(2,0);

                    std::cout << "zX = " <<zX<< std::endl;
                    std::cout << "zY = " <<zY<< std::endl;
                    std::cout << "zYaw = " <<zYaw<< std::endl;

                    // @todo: Compare with the actual odometry transformation
        //            std::cout << "xX = " <<zX<< std::endl;
        //            std::cout << "xY = " <<zY<< std::endl;
        //            std::cout << "xYaw = " <<newZ-prevZ<< std::endl;

                    // Create a constraint between ith and jth node and save measurement transform
                    Constraint c;
                    c.i = i;
                    c.j = j;
                    c.z = zTransformation;
                    Matrix3d covariance = Matrix3d::Identity(3, 3);
                    c.omega = covariance.inverse();

                    // Add constraint to the graph
                    constraints.push_back(c);
                }
            }
            std::cout << "Number of nodes in the graph = " << nodes.size() << std::endl;
            std::cout << "Number of constraints in the graph = " << constraints.size() << std::endl;

            // Perform Graph SLAM optimization whenever a loop is detected
            if(loopDetected)
//            nodes =
                    algorithm1(nodes, constraints);
        }

        //Publish Pose Array
        publishPath();

        // Publish clouds
        point_cloud_publisher_.publish(pointCloud1);
        point_cloud_publisher_second.publish(pointCloud2);

        // Update grid
        world = updateOccupancyGrid(world,laserscansaver,nodes);
        publishOccupancyGrid(world,occupub);

        // Save current pose as previous
        prevX = newX;
        prevY = newY;
        prevZ = newZ;

        // Save current scan as previous
        prevScan = currentScan;
    }
	//Update robotpublisher values
	robopub.robotxx = newX;
	robopub.robotyy = newY;
	robopub.robotyyaw = newZ;
    //robotpos(newX,newY,0,0,newZ);
}

int main(int argc, char **argv) {
	 
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	world = initializeOccupancyGrid(2000, 0.05);
	ros::Timer timer = n.createTimer(ros::Duration(5), rob_callback);
    ros::Subscriber laserSub = n.subscribe("base_scan", 100, callback);
	ros::Subscriber velSub = n.subscribe("odom", 100, vel_callback);
	robotPathPublisher = n.advertise<nav_msgs::Path> ("/path", 100, false);
	robotPosePublisher = n.advertise<geometry_msgs::PoseArray> ("/poses", 100, false);
	point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud> ("/cloud", 100,false);
	point_cloud_publisher_second = n.advertise<sensor_msgs::PointCloud> ("/cloud2", 100,false);
	point_cloud_publisher_combined = n.advertise<sensor_msgs::PointCloud> ("/cloudcombined", 100,false);
	marker_pub = n.advertise<visualization_msgs::Marker>("/lines", false);
	occupub = n.advertise<nav_msgs::OccupancyGrid> ("/world", 100,false);
	ros::spin();

	return 0;
}

