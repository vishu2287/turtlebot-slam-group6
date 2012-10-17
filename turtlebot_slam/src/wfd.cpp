#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include "nav_msgs/OccupancyGrid.h"
#include <wfd.hpp>
#include <vector>

const int MAP_open_list = 1;
const int MAP_close_list = 2;
const int FRONTIER_open_list = 3;
const int FRONTIER_close_list = 4;

// WFD implementation
	std::vector<std::vector<int> > frontierDetection(const nav_msgs::OccupancyGrid& map, int x, int y) {
		ROS_INFO("FRONTIER DETECTION START....");
		std::vector<std::vector<int> > frontierList; 
		std::vector<int> pose;
		std::vector<std::vector<int> > marker;
		for(int i = 0 ; i < map.info.width ; i++) {
			std::vector<int> row;
			for(int j = 0 ; j < map.info.height ; j++) {
				row.push_back(-1);
			}
			marker.push_back(row);
		}
		pose.push_back(x); // x-coordinate robot
		pose.push_back(y); // y-coordinate robot
		//create neccessary lists
		std::vector<std::vector<int> > queue_m;
		ROS_INFO("queue_m first push pose[0] = %i , pose[1] = %i", pose[0], pose[1]);
		queue_m.push_back(pose);
		int index1 = pose[0];
		int index2 = pose[1];
		marker[index1][index2] = MAP_open_list; 
		while (!queue_m.empty()) {
			//ROS_INFO("Loop starting");
			// dequeue
			std::vector<int> p = queue_m.front();
			queue_m.erase(queue_m.begin());
			//ROS_INFO("if map close list contains p, continue");
			//if map close list contains p, continue
			index1 = p[0];
			index2 = p[1];
			if(marker[index1][index2] == MAP_close_list) {	
				//ROS_INFO("CONTINUE!!!!! NO1");
				continue;
			}//*/
			//ROS_INFO("if(isFrontier(p[0], p[1])){");
			if (isFrontier(map, p[0], p[1])) {
				std::vector<std::vector<int> > queue_f;
				std::vector<int> newFrontier;
				queue_f.push_back(p);
				marker[index1][index2] = FRONTIER_open_list;	

		
				while (!queue_f.empty()) {
					//ROS_INFO("Second queue starting");
					std::vector<int> q = queue_f.front();
					queue_f.erase(queue_f.begin());
					index1 = q[0];
					index2 = q[1];
					if(marker[index1][index2] == MAP_close_list || marker[index1][index2] == FRONTIER_close_list) { 
						continue;
					}
					if (isFrontier(map, q[0], q[1])) {
						newFrontier.push_back(q[0]);
						newFrontier.push_back(q[1]);
						std::vector<int> neighbors = getNeighbors(map, q[0], q[1]);
						for (int i = 0; i < neighbors.size(); i += 2) {
							std::vector<int> w;
							w.push_back(neighbors[i]);
							w.push_back(neighbors[i + 1]);
							index1 = w[0];
							index2 = w[1];
							if(marker[index1][index2] != FRONTIER_open_list 
									&& marker[index1][index2] != FRONTIER_close_list
									&& marker[index1][index2] != MAP_close_list) {
								queue_f.push_back(w);
								marker[index1][index2] = FRONTIER_open_list; //frontier_open_list.push_back(w);
							}
						}
					}
					index1 = q[0];
					index2 = q[1];
					marker[index1][index2] = FRONTIER_close_list; //frontier_close_list.push_back(q);
				} // end of queue_f (loop)
				// Save data of newFrontier and mark points as map_close_list
				frontierList.push_back(newFrontier);
				//ROS_INFO("NEW FRONTIER with size %i", newFrontier.size());
				for (int i = 0; i < newFrontier.size(); i++) {
					index1 = newFrontier[i];
					index2 = newFrontier[i+1];
					marker[index1][index2] = MAP_close_list; 
					i++;
				}
			} // end of isFrontier(p)*/

			//ROS_INFO("end of isFrontier(p)");
			std::vector<int> neighbors = getNeighbors(map, p[0], p[1]);

			for (int i = 0; i < neighbors.size(); i += 2) {
				//ROS_INFO("NEIGHBORS");
				std::vector<int> v;
				v.push_back(neighbors[i]);
				v.push_back(neighbors[i + 1]);
				index1 = v[0];
				index2 = v[1];
				if(marker[index1][index2] != MAP_open_list 
					&& marker[index1][index2] != MAP_close_list) {

					std::vector<int> neighborsV = getNeighbors(map, v[0], v[1]);

					bool hasOpenSpaceNeighbor = false;

					for (int j = 0; j < neighborsV.size(); j += 2) {

						std::vector<int> neighborV;
						neighborV.push_back(neighborsV[j]);
						neighborV.push_back(neighborsV[j + 1]);

						int mapVal = getMapValue(map, neighborV[0], neighborV[1]);

						if (mapVal < 10 && mapVal != -1) {
							hasOpenSpaceNeighbor = true;
							break;
						}
					}

					if (hasOpenSpaceNeighbor) {
						queue_m.push_back(v);
						index1 = v[0];
						index2 = v[1];
						marker[index1][index2] = MAP_open_list;
					}
				}
			} // end for*/
			index1 = p[0];
			index2 = p[1];
			marker[index1][index2] = MAP_close_list;
			
		} // end of queue_m (loop)
		ROS_INFO("WFD terminated");

		return frontierList;
	}

	// checks, whether the given coordinate is a frontier point or not.
	// needs to take care of the actual positions, corners and borders
	// need to be handled carefully
	bool isFrontier(const nav_msgs::OccupancyGrid& grid, int x, int y) {
        std::vector<int> neighbors = getSurrounding(grid, x, y, 3);//*/getNeighbors(grid, x, y);
        int numExplored = 0;
        int numUnknown = 0;
		// checking neighbors
		// if there are known and unknown neighbors
		// the robot is at a frontiers
		bool unknown = false;
		bool obstacle = false;
		bool known = false;
		//ROS_INFO("for loop in isFrontier()");
		for (int i = 0; i < neighbors.size(); i += 2) {
			std::vector<int> current;
			current.push_back(neighbors[i]);
			current.push_back(neighbors[i + 1]);
			if (getMapValue(grid, current[0], current[1]) < 0) {
                numUnknown++;
				unknown = true;
            } else if (getMapValue(grid, current[0], current[1]) > 50 ){
				obstacle = true;
			}
              else {
                known = true;
                numExplored++;
            }
		}
        //return unknown && known && !obstacle;
        return(numExplored >= (neighbors.size()/4) && numUnknown >= (neighbors.size()/4) && !obstacle);
	}

    std::vector<int> getSurrounding(const nav_msgs::OccupancyGrid& grid, int x, int y, int size) {
        std::vector<int> neighbors;
        for(int i = (x-size) ; i <= (x+size) ; i++) {
            for(int j = (y-size) ; j <= (y+size) ;j++) {
                if((i - 1 >= 0) && (j - 1 >= 0) && (j + 1 < grid.info.height) && (x + 1 < grid.info.width) && (i != x && j != y)) {
                    //ROS_INFO("I = %i, J = %i", i, j);
                    neighbors.push_back(i);
                    neighbors.push_back(j);
                }
            }
        }
        return neighbors;
    }
	// gets the Neighbors as a list of vecotrs
	std::vector<int> getNeighbors(const nav_msgs::OccupancyGrid& grid, int x, int y) {

		std::vector<int> neighbors;

		if (x - 1 >= 0) {
			neighbors.push_back(x - 1);
			neighbors.push_back(y);
			if (y - 1 >= 0) {
				neighbors.push_back(x - 1);
				neighbors.push_back(y - 1);
			}
			if (y + 1 < grid.info.height) {
				neighbors.push_back(x - 1);
				neighbors.push_back(y + 1);
			}
		}
		if (x + 1 < grid.info.width) {
			neighbors.push_back(x + 1);
			neighbors.push_back(y);
			if (y - 1 >= 0) {
				neighbors.push_back(x + 1);
				neighbors.push_back(y - 1);
			}
			if (y + 1 < grid.info.width) {
				neighbors.push_back(x + 1);
				neighbors.push_back(y + 1);
			}
		}
		if (y - 1 >= 0) {
			neighbors.push_back(x);
			neighbors.push_back(y - 1);
		}
		if (y + 1 < grid.info.height) {
			neighbors.push_back(x);
			neighbors.push_back(y + 1);
		}

		return neighbors;
	}


// returns the value of position [x,y] from the occupancy grid
// the value represents the probability of an obstacle or frontier
// at position [x,y]. Values are:
// -1 = unknown
//  0 = free room
// >0 = frontier or obstacle
// maxValue = 100
	int getMapValue(const nav_msgs::OccupancyGrid& grid, int x, int y) {
		//ROS_INFO("get_map_value");
		//ROS_INFO("grid.info.width = %i , x = %i , y = %i , grid.data[%i ]", grid.info.width, x, y, (grid.info.width * y + x));
		//ROS_INFO("grid point value %f", grid.data[grid.info.width * y + x]);
		return grid.data[grid.info.width * y + x];
	}
