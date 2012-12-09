#include "ros/ros.h"
#include <Eigen/Dense>
#include <math.h>
using namespace Eigen;

//Table 11.1 Page 347 (Probabalistic Robotics)
//@TODO: Implement
MatrixXd graph_init(MatrixXd u, int deltaT) {

	MatrixXd mu((u.cols()+1)*3, 1);

	// first row, then column
	mu(0, 0) = 0;
	mu(1, 0) = 0;
	mu(2, 0) = 0;

	for (int t = 0; t < u.cols(); t++) {

		double vt = u(0, t);
		double wt = u(1, t);

		mu(3*(t+1),   0) = mu(3*t,   0) + (-vt / wt * sin(mu(3*t+2, 0)) + vt / wt * sin(mu(3*t+2, 0) + wt * deltaT));
		mu(3*(t+1)+1, 0) = mu(3*t+1, 0) + (+vt / wt * cos(mu(3*t+2, 0)) - vt / wt * cos(mu(3*t+2, 0) + wt * deltaT));
		mu(3*(t+1)+2, 0) = mu(3*t+2, 0) + (wt * deltaT);

	}
	return mu;
}
