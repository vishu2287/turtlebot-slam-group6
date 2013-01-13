#include "ros/ros.h"
#include <Eigen/Dense>
#include <Constraint.hpp>
using namespace Eigen;

std::vector<Vector3d> algorithm1 (std::vector<Vector3d> x, std::vector<Constraint> constraints) ;
