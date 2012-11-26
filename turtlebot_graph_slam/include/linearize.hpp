#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

MatrixXd linearize (MatrixXd u, std::vector<MatrixXd> z, std::vector<MatrixXd> c, MatrixXd mu, int deltaT) ;
