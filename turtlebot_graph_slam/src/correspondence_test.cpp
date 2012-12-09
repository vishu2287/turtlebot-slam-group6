#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

//Table 11.4 Page 349
//TODO Make COV_SIGMA to a list of cov sigma according to time
double correspondence_test(MatrixXd omega, VectorXd xi, VectorXd mu,
		MatrixXd sigma, int j, int k, int t) {

	/*
	 * Create tau for both features j and k
	 */
	std::vector<int> tauJK;
	int row = (t + 1 + j) * 3;
	for (int feature = 0; feature < 2; feature++) {
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
				// Add only if tauJ doesnt already contain the element
				if (std::find(tauJK.begin(), tauJK.end(), i) == tauJK.end()) {
					tauJK.push_back(i);
				}
			}
			i++;
		}
		row = (t + 1 + k) * 3;
	}

//	std::cout << "tauJK = \n";
//	for (int i = 0; i < tauJK.size(); i++) {
//		std::cout << tauJK.at(i) << std::endl;
//	}

	/*
	 * ----------------------------------------------------------
	 * LINE 2 OF ALGORITHM PAGE 364
	 * ----------------------------------------------------------
	 */

	//CONSTRUCT SIGMA Tau(j,k),Tau(j,k)
	MatrixXd sigmapart = MatrixXd::Zero(tauJK.size() * 3, tauJK.size() * 3);
	for (int z = 0; z < tauJK.size(); z++) {
		for (int x = 0; x < tauJK.size(); x++) {
			sigmapart.block(z * 3, x * 3, 3, 3) += sigma.block(tauJK[z]*3,
					tauJK[x]*3, 3, 3);
		}
	}
//	std::cout << "sigmaPart = \n" << sigmapart << std::endl;

	//CONSTRUCT Omegajk,Tau(j,k)
	MatrixXd omega_jk_tauJK = MatrixXd::Zero(6, tauJK.size() * 3);
	for (int z = 0; z < tauJK.size(); z++) {
		//Assumption the jk in Omega jk,Tau(j,k) means we want position j and k resulting in a 6x(Tau(j,k)*3) long matrix
		omega_jk_tauJK.block(0, z * 3, 3, 3) = omega.block(3*(t+1+j), tauJK[z]*3, 3, 3);
		omega_jk_tauJK.block(3, z * 3, 3, 3) = omega.block(3*(t+1+k), tauJK[z]*3, 3, 3);
	}
//	std::cout << "omega_jk_tauJK = \n" << omega_jk_tauJK << std::endl;

	//CONSTRUCT Tau(j,k),Omegajk
	MatrixXd omega_tauJK_jk = MatrixXd::Zero(tauJK.size() * 3, 6);
	for (int z = 0; z < tauJK.size(); z++) {
		omega_tauJK_jk.block(z * 3, 0, 3, 3) = omega.block(tauJK[z]*3, 3*(t+1+j), 3, 3);
		omega_tauJK_jk.block(z * 3, 3, 3, 3) = omega.block(tauJK[z]*3, 3*(t+1+k), 3, 3);
	}
//	std::cout << "omega_tauJK_jk = \n" << omega_tauJK_jk << std::endl;

	MatrixXd omega_jk_jk = MatrixXd::Zero(6, 6);
	omega_jk_jk.block(0, 0, 3, 3) += omega.block(3*(t+1+j), 3*(t+1+j), 3, 3); //Omega jk_jk = Omegaj,j ; Omega j,k ; Omega k,j; Omega k,k ??
	omega_jk_jk.block(0, 3, 3, 3) += omega.block(3*(t+1+j), 3*(t+1+k), 3, 3);
	omega_jk_jk.block(3, 0, 3, 3) += omega.block(3*(t+1+k), 3*(t+1+j), 3, 3);
	omega_jk_jk.block(3, 3, 3, 3) += omega.block(3*(t+1+k), 3*(t+1+k), 3, 3);
//	std::cout << "omega_jk_jk = \n" << omega_jk_jk << std::endl;

	MatrixXd omega_j_k = omega_jk_jk - omega_jk_tauJK * sigmapart * omega_tauJK_jk;
//	std::cout << "omega_j_k = \n" << omega_j_k << std::endl;

	/*
	 * ----------------------------------------------------------
	 * LINE 2 FINISHED
	 * ----------------------------------------------------------
	 */

	/*
	 * ----------------------------------------------------------
	 * LINE 3 OF ALGORITHM PAGE 364
	 * ----------------------------------------------------------
	 */

	//Construct mu j,k as seen on page 367
	MatrixXd omega_jk_jk_inverse = omega_jk_jk.inverse();
//	std::cout << "omega_jk_jk_inverse = \n" << omega_jk_jk_inverse << std::endl;

	VectorXd mu_taujk = VectorXd::Zero(tauJK.size() * 3);
	for (int z = 0; z < tauJK.size(); z ++) {
		mu_taujk.block(z * 3, 0, 3, 1) += mu.block(tauJK[z] * 3, 0, 3, 1);
	}
//	std::cout << "mu_taujk = \n" << mu_taujk << std::endl;

	VectorXd xi_j_k = VectorXd::Zero(6);
	row = (t + 1 + j) * 3;
	for (int z = 0; z < 2; z++) {
		xi_j_k.block(z * 3, 0, 3, 1) += xi.block(row, 0, 3, 1);
		row = (t + 1 + k) * 3;
	}
//	std::cout << "xi_j_k = \n" << xi_j_k << std::endl;

	MatrixXd mu_j_k = omega_jk_jk_inverse*(xi_j_k+omega_jk_tauJK*mu_taujk);
//	std::cout << "mu_j_k = \n" << mu_j_k << std::endl;

	xi_j_k = omega_j_k*mu_j_k;
//	std::cout << "xi_j_k = \n" << xi_j_k << std::endl;

	/*
	 * ----------------------------------------------------------
	 * LINE 3 FINISHED
	 * ----------------------------------------------------------
	 */

	/*
	 * ----------------------------------------------------------
	 * LINE 4 OF ALGORITHM PAGE 364
	 * ----------------------------------------------------------
	 */
	MatrixXd unity_one = MatrixXd::Zero(6, 3); // Why three columns?
	 unity_one <<
			 1, 0, 0,
			 0, 1, 0,
			 0, 0, 1,
			-1, 0, 0,
			 0,-1, 0,
			 0, 0,-1;

	MatrixXd omega_deltaJ_k = unity_one.transpose() * omega_j_k * unity_one;
//	std::cout << "omega_deltaJ_k = \n" << omega_deltaJ_k << std::endl;
	/*
	 * ----------------------------------------------------------
	 * LINE 4 FINISHED
	 * ----------------------------------------------------------
	 */	
	/*
	 * ----------------------------------------------------------
	 * LINE 5 OF ALGORITHM PAGE 364
	 * ----------------------------------------------------------
	 */
	VectorXd xi_deltaJ_k = unity_one.transpose()* xi_j_k;
//	std::cout << "xi_deltaJ_k = \n" << xi_deltaJ_k << std::endl;
	/*
	 * ----------------------------------------------------------
	 * LINE 5 FINISHED
	 * ----------------------------------------------------------
	 */
	/*
	 * ----------------------------------------------------------
	 * LINE 6 OF ALGORITHM PAGE 364
	 * ----------------------------------------------------------
	 */
	VectorXd mu_deltaJ_k = omega_deltaJ_k.inverse() * xi_deltaJ_k;
//	std::cout << "mu_deltaJ_k = \n" << mu_deltaJ_k << std::endl;
	/*
	 * ----------------------------------------------------------
	 * LINE 6 FINISHED
	 * ----------------------------------------------------------
	 */
	/*
	 * ----------------------------------------------------------
	 * LINE 7 OF ALGORITHM PAGE 364
	 * ----------------------------------------------------------
	 */
	MatrixXd m = 2*M_PI*omega_deltaJ_k.inverse();
	std::cout << "m = \n" << m << std::endl;

	double determinant =
			m(0,0)*m(1,1)*m(2,2)+
			m(0,1)*m(1,2)*m(2,0)+
			m(0,2)*m(1,0)*m(2,1)-
			m(0,2)*m(1,1)*m(2,2)-
			m(0,1)*m(1,0)*m(2,2)-
			m(0,0)*m(1,2)*m(2,1);
	determinant = abs(determinant); // @TODO: Check if this is correct or neccessary

	double normalizer = pow(determinant, -0.5);

	double exponent = -0.5*mu_deltaJ_k.transpose()*omega_deltaJ_k.inverse()*mu_deltaJ_k;

	double e = exp(exponent);

	return normalizer*e;

}
