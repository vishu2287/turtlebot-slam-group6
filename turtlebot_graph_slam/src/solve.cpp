#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;


//Table 11.4 Page 349
//@TODO: Implement
MatrixXd solve (MatrixXd omega_tilde, VectorXd xi_tilde, MatrixXd omega, VectorXd xi) {

	// Line 2
	MatrixXd sigma = omega_tilde.inverse();

	// Line 3
	MatrixXd mu = sigma * xi_tilde;

	std::cout << "omega_tilde = \n" << omega_tilde << std::endl;
	std::cout << "sigma = \n" << sigma << std::endl;
	std::cout << "mu = \n" << mu << std::endl;

	return mu;
}


	
