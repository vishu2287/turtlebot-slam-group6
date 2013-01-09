#include "ros/ros.h"
#include <Eigen/Dense>
#include <feature_extractor.hpp>
#include <graph_slam.hpp>
#include <OccupancyGrid.hpp>
#include <GraphBasedAlgo1.hpp>
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
using namespace Eigen;
	ros::Publisher point_cloud_publisher_;
	ros::Publisher point_cloud_publisher_second;
	ros::Publisher point_cloud_publisher_combined;
	ros::Publisher occupub;
	ros::Publisher robotPathPublisher;
//std::vector <Vector2d> odoms; //Odometry saver TODO somehow did not work on 1 system
std::vector <MatrixXd> Zs; //Z saver TODO Test if this works!
int t = 0;
int deltaT = 1;
double prevX = 0;
double prevY = 0;
double prevZ = 0;
nav_msgs::OccupancyGrid world;
nav_msgs::Path robotPath;
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
std::vector<MatrixXd> robotpossaver;
// maintain two laserscans if robot moves 0.5 meters forward
sensor_msgs::LaserScan::ConstPtr currentScan;
sensor_msgs::LaserScan::ConstPtr prevScan;
// 3 pointclouds for two different laserscans and the ICP output cloud
sensor_msgs::PointCloud pointCloud1;
sensor_msgs::PointCloud pointCloud2;
sensor_msgs::PointCloud combined;
// TUNABLE PARAMETERS
double MATCH_DISTANCE = 0.5;	// Distance after which a laser scan should be matched again
/*	Laserscancallback needed for feature extraction
--------------------------------------------------------------------------------------*/
void callback(const sensor_msgs::LaserScan::ConstPtr& msg) { // Always call graph slam for new laser readings
    /**
      Simply save the current scan.
      It will be used by the velocity callback function as seen fit.
    */
    currentScan = msg;
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
bool distance(double x1,double x2, double y1, double y2){
	double length = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
	if(length >= MATCH_DISTANCE){
		return true;
	}
	return false;
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
int velcounter = 0;
bool makesecond = false;
void vel_callback(const nav_msgs::Odometry& msg) { 

    // Get the current position of the robot
    double newX = msg.pose.pose.position.x;
    double newY = msg.pose.pose.position.y;
    geometry_msgs::Quaternion odom_quat = msg.pose.pose.orientation;
    double newZ = tf::getYaw(odom_quat);


    // If this is the first call create the first node
    if(velcounter == 0 && currentScan){

        // Save the current position as previous
        prevX = newX;
        prevY = newY;
        prevZ = newZ;

        //Save first position of the robot
        MatrixXd firstNode(3,1);
        firstNode(0,0) = newX;
        firstNode(1,0) = newY;
        firstNode(2,0) = newZ;
        robotpossaver.push_back(firstNode);

        // Save the current scan
        laserscansaver.push_back(currentScan);

        // Save the first scan as previous
        prevScan = currentScan;

        velcounter++;

        std::cout << "Origin node created" << std::endl;
        std::cout << "Number of saved poses = " << robotpossaver.size() << std::endl;
        std::cout << "Number of saved scans = " << laserscansaver.size() << std::endl;
        std::cout << "-----" << std::endl;
    }

    // If the robot has walked far enough to make the second scan
    if(distance(prevX,newX,prevY,newY) /*|| rotation(prevZ, newZ)*/) {

        // Save current scan
        laserscansaver.push_back(currentScan);

        // Save current robot pose
        MatrixXd robotPose(3,1);
        robotPose(0,0) = newX;
        robotPose(1,0) = newY;
        robotPose(2,0) = newZ;
        robotpossaver.push_back(robotPose);

        // Transform the current and last scan to PointClouds
        pointCloud1 = lasertrans(currentScan);
        pointCloud2 = lasertrans(prevScan);

        // If both clouds are not empty
        if(!pointCloud1.points.empty() || !pointCloud2.points.empty()){

            // Use scanmatch for both clouds
            MatrixXd zTransformation = scanmatch(pointCloud1,pointCloud2);

            // Get X, Y and yaw
            double zX = zTransformation(0,0);
            double zY = zTransformation(1,0);
            double zYaw = zTransformation(2,0);

            std::cout << "zX = " <<zX<< std::endl;
            std::cout << "zY = " <<zY<< std::endl;
            std::cout << "zYaw = " <<zYaw<< std::endl;
        }

        std::cout << "New node created" << std::endl;
        std::cout << "Number of saved poses = " << robotpossaver.size() << std::endl;
        std::cout << "Number of saved scans = " << laserscansaver.size() << std::endl;
        std::cout << "-----" << std::endl;

        // Publish clouds
        point_cloud_publisher_.publish(pointCloud1);
        point_cloud_publisher_second.publish(pointCloud2);

        // Update grid
        world = updateOccupancyGrid(world,laserscansaver,robotpossaver);
        publishOccupancyGrid(world,occupub);

        // Save current pose as previous
        prevX = newX;
        prevY = newY;
        prevZ = newZ;

        // Save current scan as previous
        prevScan = currentScan;
    }
    robotpos(newX,newY,0,0,newZ);
}

int main(int argc, char **argv) {
	 
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	world = initializeOccupancyGrid(2000, 0.05);
	ros::Timer timer = n.createTimer(ros::Duration(5), rob_callback);
    ros::Subscriber laserSub = n.subscribe("base_scan", 100, callback);
	ros::Subscriber velSub = n.subscribe("odom", 100, vel_callback);
	robotPathPublisher = n.advertise<nav_msgs::Path> ("/path", 100, false);
	point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud> ("/cloud", 100,false);
	point_cloud_publisher_second = n.advertise<sensor_msgs::PointCloud> ("/cloud2", 100,false);
	point_cloud_publisher_combined = n.advertise<sensor_msgs::PointCloud> ("/cloudcombined", 100,false);
	occupub = n.advertise<nav_msgs::OccupancyGrid> ("/world", 100,false);
	ros::spin();

	return 0;
}

