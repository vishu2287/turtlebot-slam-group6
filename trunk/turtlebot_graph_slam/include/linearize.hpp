#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

MatrixXd linearize (MatrixXd u, MatrixXd z, MatrixXd c, MatrixXd mu);