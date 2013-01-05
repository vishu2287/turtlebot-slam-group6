#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

MatrixXd algorithm1 (VectorXd x, std::vector<MatrixXd> e_i_j, std::vector<MatrixXd> omega_i_j) ;
