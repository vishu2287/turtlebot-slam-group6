#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

//Table 11.4 Page 349
//@TODO: Implement
MatrixXd solve(MatrixXd omega_tilde, VectorXd xi_tilde, MatrixXd omega,
		VectorXd xi) {
	ROS_INFO("CALL SOLVE METHOD");

	// Line 2
	MatrixXd sigma = omega_tilde.inverse();

	// Line 3
	VectorXd mu = MatrixXd::Zero(omega.rows(), 1);
	mu.block(0, 0, sigma.rows(), 1) = sigma * xi_tilde;
	std::cout << "mu = " << std::endl << mu << std::endl;

//	std::cout << "omega = " << std::endl << omega << std::endl;
//	std::cout << "omega rows = " << std::endl << omega << std::endl;

// @todo: Determine t
	int t = 2;
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
			if (!allZeros) {
				tauJ.push_back(i);
			}
			i++;
		}
//		std::cout << "tauJ = " << std::endl << tauJ << std::endl;
//		std::cout << "TauJ size = " << std::endl << tauJ.size() << std::endl;
//		for(int h = 0; h<tauJ.size(); h++)
//			std::cout << "Entry = " << std::endl << tauJ[h] << std::endl;

// Get Omega(j,j) inverted
		Matrix3d omegaJJ = omega.block(row, row, 3, 3);
		Matrix3d omegaJJ_inverted = omegaJJ.inverse();
//		std::cout << "row = " << row << std::endl;
//		std::cout << "omegaJJ = \n" << omegaJJ << std::endl;

		// Get Xi(j)
		Vector3d xi_j = xi.block(row, 0, 3, 1);
//		std::cout << "xi_j = \n" << xi_j << std::endl;

		// Get Omega(j,tau(j))
		MatrixXd omega_j_tauJ = MatrixXd::Zero(3, tauJ.size() * 3);
		for (int xi = 0; xi < tauJ.size(); xi++) {
			int pose = tauJ[xi];
			omega_j_tauJ.block(0, xi * 3, 3, 3) += omega.block(row, pose, 3, 3);
		}

		// Get Mu_tilde(tau(j))
		VectorXd mu_tilde_tauJ = VectorXd::Zero(tauJ.size()*3, 1);
//		std::cout << "mu_tilde_tauJ = \n" << mu_tilde_tauJ << std::endl;
		for (int xi = 0; xi < tauJ.size(); xi++) {
			int pose = tauJ[xi];
			mu_tilde_tauJ.block(xi * 3, 0, 3, 1) += mu.block(pose*3, 0, 3, 1);
		}
//		std::cout << "mu_tilde_tauJ = \n" << mu_tilde_tauJ << std::endl;

		// Line 6: Calculation
		mu.block(row, 0, 3, 1) = omegaJJ_inverted*(xi_j+omega_j_tauJ*mu_tilde_tauJ);

		j++;
	}

	std::cout << "mu = " << std::endl << mu << std::endl;

//	std::cout << "omega_tilde = \n" << omega_tilde << std::endl;
//	std::cout << "sigma = \n" << sigma << std::endl;
//	std::cout << "mu = \n" << mu << std::endl;

//	Matrix4i a = Matrix4i::Random();
//	Matrix2i b = Matrix2i::Random();
//	std::cout << "a = " << std::endl << a << std::endl;
//	std::cout << "Here is a.block(1, 1, 2, 2):" << std::endl
//			<< a.block(1, 1, 2, 2) << std::endl;
//	std::cout << "adding b :" << std::endl << b << std::endl;
//	std::cout << "to the block" << std::endl;
//	a.block(1, 1, 2, 2) += b;
//	std::cout << "New a" << std::endl << a << std::endl;

	return sigma;
}

