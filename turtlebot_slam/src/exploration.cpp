#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64MultiArray.h>
#include <list>
#include <vector>
#include <wfd.hpp>
#include <math.h>
// Include for Ocupancy Grid
#include <nav_msgs/OccupancyGrid.h>

//includes for navigation stack
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Exploration {

public:
	Exploration(ros::NodeHandle& nh) {
		// Initialize random time generator
		srand(time(NULL));

		commandPub = nh.advertise < geometry_msgs::Twist > ("cmd_vel", 1);

		mapSub = nh.subscribe("/map", 1, &Exploration::occupancyGridCallback,
				this);

		frontier_publisher = nh.advertise < sensor_msgs::PointCloud	> ("frontiers", 1);

		frontier_cloud.header.frame_id = "map";
	}

	void run() {

		MoveBaseClient ac("move_base", true);

		move_base_msgs::MoveBaseGoal goal;

		// we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -20.0;
		goal.target_pose.pose.position.y = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;
		ROS_INFO("POINT (-20,1)");
		ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		ac.waitForResult();

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved 1 meter forward");
		else
			ROS_INFO("The base failed to move forward 1 meter for some reason");

		// return 0;
	}

// get the grid from gmapping by listener
	void occupancyGridCallback(const nav_msgs::OccupancyGrid occupancyGrid) {
		grid = occupancyGrid;
		//run();
		float resolution = occupancyGrid.info.resolution;
		/*  float map_x = occupancyGrid.info.origin.position.x / resolution;
		 float map_y = occupancyGrid.info.origin.position.y / resolution;
		 float x = 0. - map_x;
		 float y = 0. - map_y;*/
		tf::TransformListener listener(ros::Duration(10));
		//transform object storing our robot's position
		tf::StampedTransform transform;
		try {
			ros::Time now = ros::Time::now();
			geometry_msgs::PointStamped base_point;
			//listener.transformPoint("odom", laser_point, base_point);
			listener.waitForTransform("map", "odom", now, ros::Duration(0.5));
			listener.lookupTransform("/map", "/odom", ros::Time(0), transform);
			// X and Y translation coordinate from the origin, where the robot started
			listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			double x = transform.getOrigin().x();
			double y = transform.getOrigin().y();
			double turn = tf::getYaw(transform.getRotation());
			//Print out current translated position of the robot
			//                   ROS_INFO(
			//                           "X Origin : %f Y Origin : %f current turnangle : %f",
			//                           x, y, turn);
			robot_pos[0] = (int) ((x / resolution) + 2000);
			robot_pos[1] = (int) ((y / resolution) + 2000);

			ROS_INFO("X = %i , Y = %i", robot_pos[0], robot_pos[1]);

		
		//               ROS_INFO(
		//                           "X Origin : %f Y Origin : %f",
		//                           x, y);
		//robot_pos[0] = x;
		//robot_pos[1] = y;
		ROS_INFO("LETS GET THE FRONTIERS");

		std::vector<std::vector<int> > frontiersList = frontierDetection(occupancyGrid, robot_pos[0], robot_pos[1]);
		ROS_INFO("FRONIERS SERVED!");
		//		ROS_INFO_STREAM("Robot pose X: " << robot_pos[0]); // x-coordinate robot
//		ROS_INFO_STREAM("Robot pose Y: " << robot_pos[1]); // y-coordinate robot

		//int num_points = frontiersList.size() / 2 //@TODO: change!
		int num_frontier_cells = 0;

		for(int i = 0 ; i < frontiersList.size() ; i++ ) {
			for(int j = 0 ; j < frontiersList[i].size() ; j++) {
				num_frontier_cells++;
			}
		}

		ROS_INFO("%i frontier cells found",num_frontier_cells);

		frontier_cloud.points.resize(num_frontier_cells);
		int frontierIndex = 0;
		for(int i = 0 ; i < frontiersList.size() ; i++ ) {
			for(int j = 0 ; j < frontiersList[i].size() ; j++) {	
			frontier_cloud.points[frontierIndex].x = (frontiersList[i][j] - 2000) * 0.05;
			frontier_cloud.points[frontierIndex].y = (frontiersList[i][j+1] - 2000) * 0.05;
//                ROS_INFO_STREAM("Frontier "<<i<<" X: "<<frontiersList[i][0]);
//                ROS_INFO_STREAM("Frontier "<<i<<" Y: "<<frontiersList[i][1]);
			frontier_cloud.points[frontierIndex].z = 0;
			frontierIndex++;
			j++;
			}
		}
		frontier_publisher.publish(frontier_cloud);
		ROS_INFO("published cloud!");
		
		//try to move to the closest frontier cell!
		/*
		int goalX = 0;
		int goalY = 0;
		double minDist = 40000.0;
		for(int i = 0 ; i < num_frontier_cells ; i++) {
			int fX = frontier_cloud.points[i].x;
			int fY = frontier_cloud.points[i].y;
			double distance = sqrt((fX-robot_pos[0])*(fX-robot_pos[0]) + (fY-robot_pos[1])*(fY-robot_pos[1]));
			if(distance < minDist) {
				minDist = distance;
				goalX = fX;
				goalY = fY;
			}
		}

		MoveBaseClient ac("move_base", true);

		move_base_msgs::MoveBaseGoal goal;

		// we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = goalX;
		goal.target_pose.pose.position.y = goalY;
		goal.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		ac.waitForResult();

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved to (%f,%f)", goalX,goalY);
		else
			ROS_INFO("The base failed to move for some reason");
//*/
		} catch (tf::TransformException& ex) {
			ROS_ERROR(
					"Received an exception trying to transform a point from \"map\" to \"odom\": %s",
					ex.what());
		}
	}

// deletes a specified element from a vector v
// @ TODO: erase(int) gibt es angeblich nicht
	void eraseEfromVector(std::vector<std::vector<int> > v,
			std::vector<int> element) {
		for (int i = 0; i < v.size(); i++) {
			if (v[i] == element) {
				v.erase(v.begin() + i);
				return;
			}
		}
	}

// @ TODO: fertig machen
	bool vec_contain(std::vector<std::vector<int> > test_vec,
			std::vector<int> comp_vec) {
		for (unsigned int i = 0; i < test_vec.size(); i++) {
			for (unsigned int z = 0; z < comp_vec.size(); z++) {
				if (comp_vec[z] != test_vec[i][z]) {
					break;
				}
				if (z == comp_vec.size() - 1) {
					return true;
				}
			}
		}

		return false;
	}

// Send a velocity command
	void move(double linearVelMPS, double angularVelRadPS) {
		geometry_msgs::Twist msg; // The default constructor will set all commands to 0
		msg.linear.x = linearVelMPS;
		msg.angular.z = angularVelRadPS;
		commandPub.publish(msg);
	}


	void spin() {
		ros::Rate rate(50); // Specify the FSM loop rate in Hz
		while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
			
			ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
			rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
		}
	}
	;



protected:
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
	ros::Subscriber mapSub; // Subscriber to the map

	int robot_pos[2];
	sensor_msgs::PointCloud frontier_cloud;
	ros::Publisher frontier_publisher;

// ---global values
// grid from gmapping
	nav_msgs::OccupancyGrid grid;
// Saved newFrontier
	
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "Exploration"); 
	ros::NodeHandle n;
	Exploration walker(n); // Create new Exploration object
	walker.spin(); // Execute FSM loop
	return 0;
}

