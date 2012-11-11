#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;


//Table 11.2 Page 347/348
//@TODO: Implement
MatrixXd linearize (MatrixXd u, MatrixXd z, MatrixXd c, MatrixXd mu) {

	MatrixXd omega = MatrixXd::Random(2,2);
	VectorXd Xi;
  	

	return omega;
}