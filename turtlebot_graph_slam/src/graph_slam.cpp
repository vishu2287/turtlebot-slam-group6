#include "ros/ros.h"
#include <Eigen/Dense>
#include <graph_init.hpp>
#include <linearize.hpp>
#include <reduce.hpp>
#include <solve.hpp>
#include <correspondence_test.hpp>
using namespace Eigen;

//Table 11.5 Page 350
//@TODO: Implement
MatrixXd graph_slam (MatrixXd u, std::vector<MatrixXd> z, int deltaT) {

	// Print of the inputs
	// std::cout << "u = \n" << u << std::endl;
	// for (int i = 0; i < z.size(); i++)
	// 	std::cout << "z[" << i << "] = \n" << z.at(i) << std::endl;

	// Time t is equal to the number of columns in u
	int t = u.cols();

	// Initialize correspondenses c with a unique value
	std::vector < MatrixXd > c;
	int uniqueValue = 0;
	for (int i = 0; i < t; i++) {
		MatrixXd newCorrespondence(1, z.at(i).cols());
		for (int col = 0; col < newCorrespondence.cols(); col++)
		{
			newCorrespondence(0, col) = uniqueValue;
			uniqueValue++;	// We assume that every observed feature belongs to a different landmark for now
		}
		c.push_back(newCorrespondence);
	}
//	for (int i = 0; i < c.size(); i++)
//		std::cout << "c[" << i << "] = " << c.at(i) << std::endl;

	// Call initialize method
	MatrixXd muPath = graph_init(u, deltaT);
	// std::cout << "muPath = \n" << muPath << std::endl;

    // Call linearize method
    MatrixXd omegaAndXi = linearize(u, z, c, muPath, deltaT);
    MatrixXd omega = omegaAndXi.topLeftCorner(omegaAndXi.rows(), omegaAndXi.rows());
    MatrixXd xi = omegaAndXi.topRightCorner(omegaAndXi.rows(),1);
    // std::cout << "omega =  \n" << omega << std::endl;
    // std::cout << "xi =  \n" << xi << std::endl;

    // // Call reduce method
    // std::vector<MatrixXd> reduceResult = reduce(omega,xi,t);
    // MatrixXd omega_tilde = reduceResult.front();
    // MatrixXd xi_tilde = reduceResult.back();
    // // std::cout << "omega_tilde = \n" << omega_tilde << std::endl;
    // // std::cout << "xi_tilde = \n" << xi_tilde << std::endl;

    // // Call solve method
    // std::vector < MatrixXd > muAndSigma = solve(omega_tilde, xi_tilde, omega, xi, t);
    // MatrixXd mu = muAndSigma.front();
    // MatrixXd sigma = muAndSigma.back();
    // std::cout << "mu = \n" << mu << std::endl;
    // std::cout << "sigma = \n" << sigma << std::endl;

//	int d = 15;
//	MatrixXd omega_tilde = MatrixXd::Random(d, d);
//	VectorXd xi_tilde = MatrixXd::Random(d, 1);
//	MatrixXd omega = MatrixXd::Random(d, d);
//	VectorXd xi = MatrixXd::Random(d, 1);
	//reduce(omega, xi);
//	MatrixXd temp = solve(omega_tilde, xi_tilde, omega, xi);
//	std::vector < MatrixXd > testo;
//	testo.push_back(temp);
//	double correspondenceProbability = correspondence_test(omega, xi, xi, temp,
//			0, 1);
//	std::cout << "correspondenceProbability = \n" << correspondenceProbability
//			<< std::endl;

	return xi; // should return full mu
}

