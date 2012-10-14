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

// Include for Ocupancy Grid
#include <nav_msgs/OccupancyGrid.h>

//includes for navigation stack
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RandomWalk {

public:
    // Construst a new RandomWalk object and hook up this ROS node
    // to the simulated robot's velocity control and laser topics
    RandomWalk(ros::NodeHandle& nh) :
        fsm(FSM_MOVE_FORWARD), rotateStartTime(ros::Time::now()), rotateDuration(
            0.f) {
        // Initialize random time generator
        srand(time(NULL));
        // Advertise a new publisher for the simulated robot's velocity command topic
        // (the second argument indicates that if multiple command messages are in
        //  the queue to be sent, only the last command will be sent)
        commandPub = nh.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
        // Subscribe to the simulated robot's laser scan topic and tell ROS to call
        // this->commandCallback() whenever a new message is published on that topic
        //laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);
        mapSub = nh.subscribe("/map", 1, &RandomWalk::occupancyGridCallback,
                              this);

        frontier_publisher = nh.advertise<sensor_msgs::PointCloud>("frontiers", 1);

        frontier_cloud.header.frame_id = "map";
        timer = nh.createTimer(ros::Duration(0.1), &RandomWalk::timerCallback, this);

        yeah = false;
    }
    void run(){

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

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");

        // return 0;
    }

    // algorithm implementation
    void frontierDetection() {
        //initialize queue_m
        //       ROS_INFO("WFD starts...");
        // queue for robot coordinates
        std::vector<int> start;
        //       ROS_INFO("start");
        // @ TODO: IST DIE POS NUN DOUBLE ODER INT??
        //       start.push_back((int)robot_pos[0]); // x-coordinate robot
        //       start.push_back((int)robot_pos[1]); // y-coordinate robot
        start.push_back((int)robot_pos[0]); // x-coordinate robot
        start.push_back((int)robot_pos[1]); // y-coordinate robot
        //    ROS_INFO("lists...");

        //create neccessary lists
        std::vector<std::vector<int> > queue_m;
        std::vector<std::vector<int> > map_open_list;
        std::vector<std::vector<int> > map_close_list;
        std::vector<std::vector<int> > frontier_open_list;
        std::vector<std::vector<int> > frontier_close_list;
        ROS_INFO("queue_m first push");
        queue_m.push_back(start);
        map_open_list.push_back(start);

        // @TODO: start aus map_close_list entfernen (falls noetig??)

        while (!queue_m.empty()) {
            //         ROS_INFO("Loop starting");
            // dequeue
            std::vector<int> p = queue_m.front();
            queue_m.erase(queue_m.begin());
            //        ROS_INFO("if map close list contains p, continue");
            //if map close list contains p, continue
            if (vec_contain(map_close_list, p)) {
                continue;
            }
            //        ROS_INFO("if(isFrontier(p[0], p[1])){");
            if(isFrontier(p[0], p[1])){
                std::vector<std::vector<int> > queue_f;
                std::vector<std::vector<int> > newFrontier;
                queue_f.push_back(p);
                frontier_open_list.push_back(p);

                // @TODO: p aus frontier_close_list entfernen (falls noetig??)
                //                       ROS_INFO("eraseFrom Vector");
                eraseEfromVector(frontier_close_list, p);

                while (!queue_f.empty()) {
                    //                          ROS_INFO("Second queue starting");

                    std::vector<int> q = queue_f.front();

                    queue_f.erase(queue_f.begin());

                    if (vec_contain(map_close_list, q)
                            && vec_contain(frontier_close_list, q)){
                        continue;
                    }

                    if (isFrontier(q[0], q[1])){

                        newFrontier.push_back(q);

                        std::vector<std::vector<int> > neighbors =
                                getNeighbors(q[0], q[1]);

                        for (int i = 0; i < neighbors.size(); i++){

                            std::vector<int> w = neighbors[i];
                            if (!vec_contain(frontier_open_list, w) &&
                                    !vec_contain(frontier_close_list, w) &&
                                    !vec_contain(map_close_list, w)){
                                queue_f.push_back(w);
                                frontier_open_list.push_back(w);

                                // @TODO: w aus frontier_close_list entfernen (falls noetig??)
                                eraseEfromVector(frontier_close_list, w);
                            }
                        }
                    }
                    frontier_close_list.push_back(q);

                    // @TODO: q aus frontier_open_list entfernen
                    eraseEfromVector(frontier_open_list, q);
                } // end of queue_f (loop)

                // Save data of newFrontier and mark points as map_close_list
                for (int i = 0; i < newFrontier.size(); i++) {
                    frontiersList.push_back(std::vector<int>(newFrontier[i])); //@TODO: TESTEN!
                    map_close_list.push_back(newFrontier[i]);

                    // @TODO: newFrontier[i] aus map_open_list entfernen

                    eraseEfromVector(map_open_list, newFrontier[i]);
                }
            } // end of isFrontier(p)

            //           ROS_INFO("end of isFrontier(p)");
            std::vector<std::vector<int> > neighbors = getNeighbors(p[0], p[1]);

            for (int i = 0; i < neighbors.size(); i++){

                std::vector<int> v = neighbors[i];
                if (!vec_contain(map_open_list, v) &&

                        !vec_contain(map_close_list, v)){

                    std::vector<std::vector<int> > neighborsV =
                            getNeighbors(v[0], v[1]);

                    bool hasOpenSpaceNeighbor = false;

                    for (int j = 0; j < neighborsV.size(); j++){

                        std::vector<int> neighborV = neighborsV[j];

                        int mapVal = getMapValue(neighborV[0], neighborV[1]);

                        if (mapVal < 10 && mapVal != -1){
                            hasOpenSpaceNeighbor = true;
                            break;
                        }
                    }

                    if (hasOpenSpaceNeighbor){
                        queue_m.push_back(v);
                        map_open_list.push_back(v);
                        eraseEfromVector(map_close_list, v); //@TODO: ??
                    }
                }
            }
            map_close_list.push_back(p);
            eraseEfromVector(map_open_list, p); //@TODO: ??
        } // end of queue_m (loop)
        ROS_INFO("WFD terminated");
        ROS_INFO_STREAM("Robot pose X: "<<robot_pos[0]); // x-coordinate robot
        ROS_INFO_STREAM("Robot pose Y: "<<robot_pos[1]); // y-coordinate robot

        int num_points = frontiersList.size()/2;
        frontier_cloud.points.resize(num_points);
        int pointI = 0;
        for(int i = 0; i < num_points; i++) {

                frontier_cloud.points[i].x = (frontiersList[i][0]-2000)*0.05;
                frontier_cloud.points[i].y = (frontiersList[i][1]-2000)*0.05;
                ROS_INFO_STREAM("Frontier "<<i<<" X: "<<frontiersList[i][0]);
                ROS_INFO_STREAM("Frontier "<<i<<" Y: "<<frontiersList[i][1]);
                frontier_cloud.points[i].z = 0;
                pointI++;
        }
        frontier_publisher.publish(frontier_cloud);
        ROS_INFO("published cloud!");


    }

    // checks, whether the given coordinate is a frontier point or not.
    // needs to take care of the actual positions, corners and borders
    // need to be handled carefully
    bool isFrontier(int x, int y) {
        std::vector<std::vector<int> > neighbors = getNeighbors(x, y);

        // checking neighbors
        // if there are known and unknown neighbors
        // the robot is at a frontiers
        bool unknown = false;

        bool known = false;
        //ROS_INFO("for loop in isFrontier()");
        for (int i = 0; i < neighbors.size(); i++) {
            std::vector<int> current = neighbors[i];
            if (getMapValue(current[0], current[1]) < 0) {
               // ROS_INFO("if loop in for isFrontier()");
                unknown = true;
            }
            else
                known = true;
        }
        //ROS_INFO("return unknown && known");
        return unknown && known;
    }

    // gets the Neighbors as a list of vecotrs
    std::vector<std::vector<int> > getNeighbors(int x, int y) {

        std::vector<std::vector<int> > neighbors;

        if (x - 1 >= 0) {
            neighbors.push_back(std::vector<int> { x - 1, y });
        if (y - 1 >= 0) {
            neighbors.push_back(std::vector<int> { x - 1, y - 1 });
    }
    if (y + 1 < grid.info.height) {
        neighbors.push_back(std::vector<int> { x - 1, y + 1 });
}
}
if (x + 1 < grid.info.width) {
    neighbors.push_back(std::vector<int> { x + 1, y });
if (y - 1 >= 0) {
    neighbors.push_back(std::vector<int> { x + 1, y - 1 });
}
if (y + 1 < grid.info.width) {
    neighbors.push_back(std::vector<int> { x + 1, y + 1 });
}
}
if (y - 1 >= 0) {
    neighbors.push_back(std::vector<int> { x, y - 1 });
}
if (y + 1 < grid.info.height) {
    neighbors.push_back(std::vector<int> { x, y + 1 });
}
return neighbors;
}

// get the grid from gmapping by listener
void occupancyGridCallback(const nav_msgs::OccupancyGrid occupancyGrid) {
    grid = occupancyGrid;
    //run();
   /* float resolution = occupancyGrid.info.resolution;
    float map_x = occupancyGrid.info.origin.position.x / resolution;
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
        listener.waitForTransform("map", "odom", now,
                                  ros::Duration(0.5));
        listener.lookupTransform("/map", "/odom", ros::Time(0),
                                 transform);
        // X and Y translation coordinate from the origin, where the robot started
        listener.lookupTransform("/odom", "/base_link",
                                 ros::Time(0), transform);
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double turn = tf::getYaw(transform.getRotation());
        //Print out current translated position of the robot
        //                   ROS_INFO(
        //                           "X Origin : %f Y Origin : %f current turnangle : %f",
        //                           x, y, turn);
        robot_pos[0] = (int)((x/0.05)+2000);
        robot_pos[1] = (int)((y/0.05)+2000);
        //robot_pos[2] = turn;
    } catch (tf::TransformException& ex) {
        ROS_ERROR(
                    "Received an exception trying to transform a point from \"map\" to \"odom\": %s",
                    ex.what());
    }
    //               ROS_INFO(
    //                           "X Origin : %f Y Origin : %f",
    //                           x, y);
    //robot_pos[0] = x;
    //robot_pos[1] = y;
    frontierDetection();
    yeah = true;
}

// returns the value of position [x,y] from the occupancy grid
// the value represents the probability of an obstacle or frontier
// at position [x,y]. Values are:
// -1 = unknown
//  0 = free room
// >0 = frontier or obstacle
// maxValue = 100
int getMapValue(int x, int y) {
    //ROS_INFO("get_map_value");
    //ROS_INFO("grid.info.width = %i , x = %i , y = %i , grid.data[%i ]", grid.info.width, x, y, (grid.info.width * y + x));
    //ROS_INFO("grid point value %f", grid.data[grid.info.width * y + x]);
    return grid.data[grid.info.width * y + x];
}

// deletes a specified element from a vector v
// @ TODO: erase(int) gibt es angeblich nicht
void eraseEfromVector(std::vector<std::vector<int> > v,
                      std::vector<int> element) {
    for (int i = 0; i < v.size(); i++) {
        if (v[i] == element) {
            v.erase(v.begin()+i);
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

///////////////////////////
// TRANSFORMATION METHOD //
///////////////////////////

bool yeah;
void timerCallback(const ros::TimerEvent& event) {
    tf::TransformListener listener(ros::Duration(10));
    //transform object storing our robot's position
    tf::StampedTransform transform;
    try {
        ros::Time now = ros::Time::now();
        geometry_msgs::PointStamped base_point;
        //listener.transformPoint("odom", laser_point, base_point);
        listener.waitForTransform("map", "odom", now,
                                  ros::Duration(0.5));
        listener.lookupTransform("/map", "/odom", ros::Time(0),
                                 transform);
        // X and Y translation coordinate from the origin, where the robot started
        listener.lookupTransform("/odom", "/base_link",
                                 ros::Time(0), transform);
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double turn = tf::getYaw(transform.getRotation());
        //Print out current translated position of the robot
        //                   ROS_INFO(
        //                           "X Origin : %f Y Origin : %f current turnangle : %f",
        //                           x, y, turn);
        robot_pos[0] = (int)((x/0.05)+2000);
        robot_pos[1] = (int)((y/0.05)+2000);
        robot_pos[2] = turn;
    } catch (tf::TransformException& ex) {
        ROS_ERROR(
                    "Received an exception trying to transform a point from \"map\" to \"odom\": %s",
                    ex.what());
    }
    //run();
    //               if(!yeah)
    //               {

    //               }
    //ROS_INFO("Moving to...");
}



// Main FSM loop for ensuring that ROS messages are
// processed in a timely manner, and also for sending
// velocity controls to the simulated robot based on the FSM state
void spin() {
    ros::Rate rate(50); // Specify the FSM loop rate in Hz
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
        // TODO: Either call:
        //
        //       - move(0, ROTATE_SPEED_RADPS); // Rotate right
        //
        //       or
        //
        //       - move(FORWARD_SPEED_MPS, 0); // Move foward
        //
        //       depending on the FSM state; also change the FSM state when appropriate
        /////////////////////// ANSWER CODE BEGIN ///////////////////


        /////////////////////// ANSWER CODE END ///////////////////
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
}
;

enum FSM {
    FSM_MOVE_FORWARD, FSM_ROTATE
};
// Tunable parameters

// TODO: tune parameters as you see fit
const static double MIN_SCAN_ANGLE_RAD = -15.0 / 180 * M_PI;
const static double MAX_SCAN_ANGLE_RAD = +15.0 / 180 * M_PI;
const static float PROXIMITY_RANGE_M = 0.8; // Should be smaller than sensor_msgs::LaserScan::range_max
const static double FORWARD_SPEED_MPS = 0.75;
const static double ROTATE_SPEED_RADPS = M_PI / 2;

protected:
ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
ros::Subscriber mapSub; // Subscriber to the map
enum FSM fsm; // Finite state machine for the random walk algorithm
ros::Time rotateStartTime; // Start time of the rotation
ros::Duration rotateDuration; // Duration of the rotation
ros::Timer timer;
ros::Subscriber occupSub; // OCCUPANCY GRID Subscriber
double robot_pos[3];
sensor_msgs::PointCloud frontier_cloud;
ros::Publisher frontier_publisher;

// ---global values
// grid from gmapping
nav_msgs::OccupancyGrid grid;
// Saved newFrontier
std::vector<std::vector<int> > frontiersList;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
    ros::NodeHandle n;

    RandomWalk walker(n); // Create new random walk object
    walker.spin(); // Execute FSM loop
    return 0;
}

