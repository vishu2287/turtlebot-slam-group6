#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include <float.h>
#include <math.h>
using namespace Eigen;

MatrixXd graph_slam (MatrixXd u, std::vector<MatrixXd> z, int deltaT);


	
