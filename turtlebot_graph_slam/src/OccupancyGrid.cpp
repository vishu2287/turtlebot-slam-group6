#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <Eigen/Dense>
#include "nav_msgs/OccupancyGrid.h"
using namespace Eigen;

nav_msgs::OccupancyGrid initializeOccupancyGrid(int SIZE, double resolution) {
    nav_msgs::OccupancyGrid og;
    og.info.resolution = resolution;
    og.header.frame_id = "/world";
    og.info.origin.position.x = -SIZE/2;
    og.info.origin.position.y = -SIZE/2;
    og.header.stamp = ros::Time::now();
    og.info.width = SIZE;
    og.info.height = SIZE;
    og.data.resize(SIZE * SIZE);

    return og;
}

nav_msgs::OccupancyGrid initializeOccupancyGridDefault() {
    return initializeOccupancyGrid(4000, 0.5);
}

/* not working, max does not apply for double
// make sure to take resolution into account if using this
int calculateFakeSize(VectorXd mu) {
    int xPlus = 0, xMinus = 0;
    int yPlus = 0, yMinus = 0;
    for (int position = 0; position < mu.rows(); position+=3) {
        xPlus = std::max(xPlus, mu(0, position));
        yPlus = std::max(yPlus, mu(0, position + 1));

        xMinus = std::min(xMinus, mu(position));
        yMinus = std::min(yMinus, mu(position));
    }

    int SIZE = std::max(xPlus - xMinus, yPlus - yMinus) + 100; // make it quadratic and + 100 just to be save
    return SIZE;
}
*/

nav_msgs::OccupancyGrid updateOccupancyGrid(nav_msgs::OccupancyGrid og, VectorXd mu, int numberOfPoses, int erase) {

    /*		Populate a Occupancy GridFF
    --------------------------------------------------------------------------------------*/
    // Collecting maximal and minimal known x and y for size of the occupancy grid

    if(erase == 1) {
        // TODO erase, but how?
        //og.data.erase(0, og.info.width * og.info.height);
    }

    // robot path does not matter
    // landmarks theta is not of any use either

    // @TODO FIX!!
    // seems to be a problem in the loop head, mayme the mu.rows()
    for (int pose = numberOfPoses * 3; pose < mu.rows(); pose += 3) {
        std::cout << "calc x" <<std::endl;
        int xx = (int)(mu(pose) * og.info.resolution);
        std::cout << xx <<std::endl;

        std::cout << "calc y" <<std::endl;
        int yy = (int)(mu(pose + 1) * og.info.resolution * og.info.width);
        std::cout << yy << std::endl;

        og.data[xx + yy] = 99; // between 1 and 100
    }
    return og;
}
