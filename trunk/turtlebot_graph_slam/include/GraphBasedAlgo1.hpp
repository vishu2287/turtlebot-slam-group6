#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

std::vector<Vector3d> algorithm1 (std::vector<Vector3d> x, std::vector<Vector3d> z, std::vector<Matrix3d> omegas, std::vector<int> is, std::vector<int> js) ;
