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

//Tell the navigation stack where to run to
void run(double x, double y, double turn){
	//@TODO make a turn

          //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
  	  ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 2.0;

  ROS_INFO_STREAM("Moving to position x:"<<goal.target_pose.pose.position.x<<" y:"<<goal.target_pose.pose.position.y);
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Robot moved to position x:"<< goal.target_pose.pose.position.x<<" y:"<<goal.target_pose.pose.position.y);
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
 }

// get the grid from gmapping by listener
	void occupancyGridCallback(const nav_msgs::OccupancyGrid occupancyGrid) {
		grid = occupancyGrid;		
		float resolution = occupancyGrid.info.resolution;
		tf::TransformListener listener(ros::Duration(10));
		//transform object storing our robot's position
		tf::StampedTransform transform;
		try {
			ros::Time now = ros::Time::now();
			geometry_msgs::PointStamped base_point;
			listener.waitForTransform("/map", "/base_link", now, ros::Duration(0.5));
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			double x = transform.getOrigin().x();
			double y = transform.getOrigin().y();
			double turn = tf::getYaw(transform.getRotation());
			ROS_INFO_STREAM("STARTPOSITION X: " <<x<<" STARTPOSITION Y: " <<y);
			robot_pos[0] = (int) ((x / resolution) + (int)(occupancyGrid.info.width/2));
			robot_pos[1] = (int) ((y / resolution) + (int)(occupancyGrid.info.height/2));
			ROS_INFO("X = %i , Y = %i", robot_pos[0], robot_pos[1]);
			ROS_INFO("LETS GET THE FRONTIERS");
			std::vector<std::vector<int> > frontiersList = frontierDetection(occupancyGrid, robot_pos[0], robot_pos[1]);
			ROS_INFO("FRONIERS SERVED!");


		//int num_points = frontiersList.size() / 2 //@TODO: change!
		int num_frontier_cells = 0;

			for(int i = 0 ; i < frontiersList.size() ; i++ ) {
				for(int j = 0 ; j < frontiersList[i].size() ; j++) {
					num_frontier_cells++;
				}
			}

			ROS_INFO("%i frontier cells found",num_frontier_cells);
			double goalX = 0;
			double goalY = 0;
			frontier_cloud.points.resize(num_frontier_cells);
			int frontierIndex = 0;
			//arbitrary value which is always higher than any distance found
			double minDist = 40000.0;
			//fill frontier cloud for publishing and calculate frontier closest to robot base
			for(int i = 0 ; i < frontiersList.size() ; i++ ) {
				for(int j = 0 ; j < frontiersList[i].size() ; j++) {	
				double fX = (frontiersList[i][j] - (int)(occupancyGrid.info.width/2)) * occupancyGrid.info.resolution;
				frontier_cloud.points[frontierIndex].x = fX;
				double fY = (frontiersList[i][j+1] - (int)(occupancyGrid.info.height/2)) * occupancyGrid.info.resolution;
				frontier_cloud.points[frontierIndex].y = fY;
				double distance = sqrt(fX*fX+fY*fY);
				frontier_cloud.points[frontierIndex].z = 0;
				frontierIndex++;
				j++;
				if(distance < minDist) {
					minDist = distance;
					goalX = fX;
					goalY = fY;
				}
				}
			}
			frontier_publisher.publish(frontier_cloud);
			ROS_INFO("published cloud!");
			run(goalX,goalY,turn);

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

