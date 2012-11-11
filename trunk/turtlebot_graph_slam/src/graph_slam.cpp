#include "ros/ros.h"
#include <Eigen/Dense>
#include <graph_init.hpp>
#include <linearize.hpp>
#include <reduce.hpp>
#include <solve.hpp>
using namespace Eigen;


//Table 11.5 Page 350
//@TODO: Implement
MatrixXd graph_slam (MatrixXd u, MatrixXd z, MatrixXd c) {

	MatrixXd mu = MatrixXd::Random(2,2);

	return mu;
}


	
