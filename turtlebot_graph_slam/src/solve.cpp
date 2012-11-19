#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

//Table 11.4 Page 349
//@TODO: Implement
MatrixXd solve(MatrixXd omega_tilde, VectorXd xi_tilde, MatrixXd omega,
		VectorXd xi) {

	// Line 2
	MatrixXd sigma = omega_tilde.inverse();

	// Line 3
	MatrixXd mu = sigma * xi_tilde;

	// @todo: Determine t
	int t = 3;
	int j = 0;
	for (int row = (t + 1) * 3; row < omega.rows(); row += 3) {

		// Line 5: Populate tau(j) with all pose indices where feature j was observed
		std::vector<int> tauJ;
		int i = 0;
		for (int column = 0; column < (t + 1) * 3; column += 3) {
			bool allZeros = true;
			for (int featureRow = row; featureRow < row + 3; featureRow++) {
				for (int featureCol = column; featureCol < column + 3;
						featureCol++) {
					if (omega(featureRow, featureCol) != 0) {
						allZeros = false;
						break;
					}
				}
				if (!allZeros) {
					break;
				}
			}
			if (allZeros) {
				tauJ.push_back(i);
			}
			i++;
		}

		// Get Omega j,j

		//
		j++;
	}

	std::cout << "omega_tilde = \n" << omega_tilde << std::endl;
	std::cout << "sigma = \n" << sigma << std::endl;
	std::cout << "mu = \n" << mu << std::endl;

	return mu;
}

