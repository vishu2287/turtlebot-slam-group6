#include "ros/ros.h"
#include <Eigen/Dense>
#include <math.h>
using namespace Eigen;

//Table 11.1 Page 347 (Probabalistic Robotics)
//@TODO: Implement
MatrixXd graph_init(MatrixXd u) {

	// @todo: Check deltaT
	int deltaT = 1;

	MatrixXd mu(3, u.cols() + 1);

	// first row, then column
	mu(0, 0) = 0;
	mu(1, 0) = 0;
	mu(2, 0) = 0;

	for (int t = 0; t < u.cols(); t++) {

		double vt = u(0, t);
		double wt = u(1, t);

		mu(0, t + 1) = mu(0, t) + (-vt / wt * sin(mu(2, t)) + vt / wt * sin(mu(2, t) + wt * deltaT));
		mu(1, t + 1) = mu(1, t) + (+vt / wt * cos(mu(2, t)) - vt / wt * cos(mu(2, t) + wt * deltaT));
		mu(2, t + 1) = mu(2, t) + (wt * deltaT);

	}
	//std::cout << "Here is the matrix u:\n" << u << std::endl;
	//std::cout << "Here is the matrix mu:\n" << mu << std::endl;
	std::cout << "mu = \n" << mu << std::endl;
	return mu;
}
