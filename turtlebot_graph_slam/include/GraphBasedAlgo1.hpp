#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

MatrixXd algorithm1 (VectorXd x, std::vector<Vector3d> z, std::vector<Matrix3d> omegas, std::vector<int> is, std::vector<int> js) ;
