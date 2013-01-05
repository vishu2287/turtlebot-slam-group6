#include "ros/ros.h"
#include <float.h>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

MatrixXd algorithm1(VectorXd x, std::vector<MatrixXd> e_i_j, std::vector<MatrixXd> omega_i_j) {
	// find the maximum likelihood solution
	bool converged = false;
	while(!converged)
	{
		VectorXd b = VectorXd::Zero(3,1);
		MatrixXd H = MatrixXd::Zero(3,3);

	}
	MatrixXd H_star = MatrixXd::Zero(3,3);
	return H_star;
}

