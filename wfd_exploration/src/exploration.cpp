#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
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
        robotX = 0;
        robotY = 0;
        commandPub = nh.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
        onTheMove = false;
        mapSub = nh.subscribe("/world", 1, &Exploration::occupancyGridCallback,
                              this);
        velSub = nh.subscribe("odom", 5, &Exploration::vel_callback, this);
        frontier_publisher = nh.advertise < sensor_msgs::PointCloud> ("frontiers", 1);
        next_frontier_publisher = nh.advertise < sensor_msgs::PointCloud> ("next_frontier", 1);
        next_frontier.header.frame_id = "/world";
        frontier_cloud.header.frame_id = "/world";
    }

    //Tell the navigation stack where to run to
    void run(double x, double y){
        onTheMove = true;
        MoveBaseClient ac("move_base", true);

        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "\world";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = 1;

        ROS_INFO_STREAM("Moving to position x:"<<goal.target_pose.pose.position.x<<" y:"<<goal.target_pose.pose.position.y);
        ac.sendGoal(goal);

        ac.waitForResult(ros::Duration(3));

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO_STREAM("Robot moved to position x:"<< goal.target_pose.pose.position.x<<" y:"<<goal.target_pose.pose.position.y);
        else
            ROS_INFO_STREAM("The base failed to move to position x:"<< goal.target_pose.pose.position.x<<" y:"<<goal.target_pose.pose.position.y);
        onTheMove = false;
    }

    void vel_callback(const nav_msgs::Odometry& msg) {

        // Get the current position of the robot
        robotX = msg.pose.pose.position.x;
        robotY = msg.pose.pose.position.y;
//        ROS_INFO_STREAM("Robot position : ("<< robotX <<", "<<robotY<<").");
    }

    // get the grid from gmapping by listener
    double prevGoalX;
    double prevGoalY;
    std::vector<std::vector<double> > blocked;
    int sameCounter;

    bool isBlocked(double x, double y)
    {
        for(int i = 0; i<blocked.size(); i++)
        {
            if(blocked[i][0] == x && blocked[i][1] == y)
                return true;
        }
        return false;
    }

    void occupancyGridCallback(const nav_msgs::OccupancyGrid occupancyGrid) {
        if(!onTheMove) {
            onTheMove = true;
            grid = occupancyGrid;
            float resolution = occupancyGrid.info.resolution;
            tf::TransformListener listener(ros::Duration(10));
            //transform object storing our robot's position
            tf::StampedTransform transform;
            try {
                ros::Time now = ros::Time::now();
                geometry_msgs::PointStamped base_point;
                listener.waitForTransform("/world", "/base_link", now, ros::Duration(0.5));
                listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
                double x = transform.getOrigin().x();
                double y = transform.getOrigin().y();

//                ROS_INFO_STREAM("STARTPOSITION X: " <<x<<" STARTPOSITION Y: " <<y);
                robot_pos[0] = (int) ((x / resolution) + (int)(occupancyGrid.info.width/2));
                robot_pos[1] = (int) ((y / resolution) + (int)(occupancyGrid.info.height/2));
//                ROS_INFO("X = %i , Y = %i", robot_pos[0], robot_pos[1]);
//                ROS_INFO("LETS GET THE FRONTIERS");
                std::vector<std::vector<int> > frontiersList = frontierDetection(occupancyGrid, robot_pos[0], robot_pos[1]);
//                ROS_INFO("FRONIERS SERVED!");

                x = robotX;
                y = robotY;

                int num_frontier_cells = 0;

                for(unsigned int i = 0 ; i < frontiersList.size() ; i++ ) {
                    for(unsigned int j = 0 ; j < frontiersList[i].size() ; j++) {
                        num_frontier_cells++;
                    }
                }

//                ROS_INFO("%i frontier cells found",num_frontier_cells);
                double goalX = 0;
                double goalY = 0;
                frontier_cloud.points.resize(0);
                frontier_cloud.points.resize(num_frontier_cells);
                int frontierIndex = 0;
                //arbitrary value which is always higher than any distance found
                double minDist = 40000.0;
                //represents the closest frontier
                std::vector <int> closestFrontier;

                int fewestObstacles = 999999;

//                std::vector<std::vector<int> > blocked;

                //fill frontier cloud for publishing and calculate frontier closest to robot base
                for(unsigned int i = 0 ; i < frontiersList.size() ; i++ ) {
                    for(unsigned int j = 0 ; j < frontiersList[i].size() ; j++) {
                        double fX = (frontiersList[i][j] - (int)(occupancyGrid.info.width/2)) * occupancyGrid.info.resolution;
                        frontier_cloud.points[frontierIndex].x = fX;
                        double fY = (frontiersList[i][j+1] - (int)(occupancyGrid.info.height/2)) * occupancyGrid.info.resolution;


                        frontier_cloud.points[frontierIndex].y = fY;
                        frontier_cloud.points[frontierIndex].z = 0;
                        frontierIndex++;
                        j++;



                        if(!isBlocked(fX, fY))
                        {
                        double gridX = (fX / occupancyGrid.info.resolution) + (int)(occupancyGrid.info.width/2);
                        double gridY = (fY / occupancyGrid.info.resolution) + (int)(occupancyGrid.info.height/2);

                        // Get neighbors
                        std::vector<int> neighbors = getSurrounding(occupancyGrid, gridX, gridY, 20);

                        // Count obstacles
                        int obstacles = 0;
                        double minObstacleDist = 9999;
                        for (int a = 0; a < neighbors.size(); a += 2) {
                            std::vector<int> current;
                            current.push_back(neighbors[a]);
                            current.push_back(neighbors[a + 1]);
                            if (getMapValue(occupancyGrid, current[0], current[1]) > 0) {
                                obstacles++;
                                double obstacleDist = sqrt((fX-current[0])*(fX-current[0])+(fY-current[1])*(fY-current[1]));
                                if(obstacleDist < minObstacleDist)
                                    minObstacleDist = obstacleDist;
                            }
                        }

                        double distanceToRobot = sqrt((fX-x)*(fX-x)+(fY-y)*(fY-y));
                        double distanceToLastGoal = sqrt((fX-prevGoalX)*(fX-prevGoalX)+(fY-prevGoalY)*(fY-prevGoalY));



                        obstacles += 5*distanceToLastGoal+distanceToRobot;

                        if(obstacles < fewestObstacles && minObstacleDist > 0.2)
                        {
                            fewestObstacles = obstacles;
                            double randomX = 1./(rand() % 100 + 1);
                            double randomY = 1./(rand() % 100 + 1);
                            double randomRange = 1;
                            goalX = fX;//-randomRange/2+randomRange*randomX;
                            goalY = fY;//-randomRange/2+randomRange*randomY;

//                            std::cout << "distanceToRobot = " << distanceToRobot << std::endl;
//                            std::cout << "distanceToLastGoal = " << distanceToLastGoal << std::endl;
//                            std::cout << "obstacles = " << obstacles << std::endl;
                        }

//                        if(distance < minDist && distance > 2) {
//                            closestFrontier = frontiersList[i];
//                            minDist = distance;
//                            goalX = fX;
//                            goalY = fY;
//                        }
                        }
                    }
                }
                if(goalX == prevGoalX && goalY == prevGoalY)
                    sameCounter++;
                else
                    sameCounter = 0;
                if(sameCounter > 0)
                {
                    std::vector<double> error;
                    error.push_back(goalX);
                    error.push_back(goalY);
                    blocked.push_back(error);
                }
                prevGoalX = goalX;
                prevGoalY = goalY;
//                ROS_INFO_STREAM();
                // find a reachable goal position
                // select a target cell surrounded by free space only
//                double gridX = (goalX / occupancyGrid.info.resolution) + (int)(occupancyGrid.info.width/2);
//                double gridY = (goalY / occupancyGrid.info.resolution) + (int)(occupancyGrid.info.height/2);
//                std::vector<int> neighbors = getSurrounding(occupancyGrid, gridX, gridY, 50);
//                if(!isFree(neighbors, occupancyGrid)) {
//                    for (int i = 0; i < neighbors.size(); i += 2) {
//                        std::vector<int> current;
//                        current.push_back(neighbors[i]);
//                        current.push_back(neighbors[i + 1]);
//                        std::vector<int> currentNeighbors = getSurrounding(occupancyGrid, current[0], current[1], 10);
//                        if(isFree(currentNeighbors, occupancyGrid)) {
//                            goalX = (current[0] - (int)(occupancyGrid.info.width/2)) * occupancyGrid.info.resolution;
//                            goalY = (current[1] - (int)(occupancyGrid.info.width/2)) * occupancyGrid.info.resolution;
//                            break;
//                        }
//                    }
//                }

                next_frontier.points.resize(1);
                next_frontier.points[0].x = goalX;
                next_frontier.points[0].y = goalY;
                next_frontier.points[0].z = 0;

                frontier_publisher.publish(frontier_cloud);
                next_frontier_publisher.publish(next_frontier);
//                ROS_INFO("published cloud!");
                run(goalX,goalY);

            } catch (tf::TransformException& ex) {
                ROS_ERROR(
                            "Received an exception trying to transform a point from \"map\" to \"odom\": %s",
                            ex.what());
            }
        }
    }

    bool isFree(std::vector<int> cells, const nav_msgs::OccupancyGrid& occupancyGrid) {
        for (int i = 0; i < cells.size(); i += 2) {
            std::vector<int> current;
            current.push_back(cells[i]);
            current.push_back(cells[i + 1]);
            if (getMapValue(occupancyGrid, current[0], current[1]) <=0) {
                return false;
            }
        }
        return true;
    }

    // Send a velocity command
    void move(double linearVelMPS, double angularVelRadPS) {
        geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        msg.linear.x = linearVelMPS;
        msg.angular.z = angularVelRadPS;
        commandPub.publish(msg);
    }


    void spin() {
        ros::Rate rate(10); // Specify the FSM loop rate in Hz
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
    ros::Subscriber velSub; // get the position of the robot.
    bool onTheMove;
    double robotX;
    double robotY;
    int robot_pos[2];
    sensor_msgs::PointCloud frontier_cloud;
    sensor_msgs::PointCloud next_frontier;
    ros::Publisher frontier_publisher;
    ros::Publisher next_frontier_publisher;

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

