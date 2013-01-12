#include "ros/ros.h"
#include <float.h>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

std::vector<Vector3d> algorithm1(std::vector<Vector3d> x, std::vector<Vector3d> z, std::vector<Matrix3d> omegas, std::vector<int> is, std::vector<int> js) {

	// find the maximum likelihood solution
	bool converged = false;

//	VectorXd b(x.rows(),1);
//	MatrixXd H(x.rows(),x.rows());

//	while(!converged)
//	{
        VectorXd b = VectorXd::Zero(3*x.size(),1);
        MatrixXd H = MatrixXd::Zero(3*x.size(),3*x.size());
		for(int constraintIndex = 0; constraintIndex < z.size(); constraintIndex++)
		{
//            std::cout << "constraintIndex = \n" << constraintIndex << std::endl;

			// Get the indices of the nodes this constraint refers to
			int i = is[constraintIndex];
			int j = js[constraintIndex];

//            std::cout << "i = \n" << i << std::endl;
//            std::cout << "j = \n" << j << std::endl;

            Vector3d x_i = x[i];		// (28)
            Vector3d x_j = x[j];
			Vector3d z_i_j = z[constraintIndex];		// (29)

//            std::cout << "x_i = \n" << x_i << std::endl;
//            std::cout << "x_j = \n" << x_j << std::endl;
//            std::cout << "z_i_j = \n" << z_i_j << std::endl;

			Vector2d t_i = x_i.block(0, 0, 2, 1);		// Position vector of node i
			Vector2d t_j = x_j.block(0, 0, 2, 1);		// Position vector of node j
			Vector2d t_i_j = z_i_j.block(0, 0, 2, 1);	// Measured position difference vector between nodes i and j

//            std::cout << "t_i = \n" << t_i << std::endl;
//            std::cout << "t_j = \n" << t_j << std::endl;
//            std::cout << "t_i_j = \n" << t_i_j << std::endl;

			double phi_i = x_i(2,0);					// Angle of node i
			double phi_j = x_j(2,0);					// Angle of node j
			double phi_i_j = z_i_j(2,0);				// Measured angle between nodes i and j

//			std::cout << "phi_i = \n" << phi_i << std::endl;
//			std::cout << "phi_j = \n" << phi_j << std::endl;
//			std::cout << "phi_i_j = \n" << phi_i_j << std::endl;

			Matrix2d R_i(2,2);							// Rotation matrix for angle of node i
			R_i(0,0) =  cos(phi_i);
			R_i(0,1) = -sin(phi_i);
			R_i(1,0) =  sin(phi_i);
			R_i(1,1) =  cos(phi_i);

			Matrix2d R_i_j(2,2);						// Rotation matrix for measured angle between nodes i and j
			R_i_j(0,0) =  cos(phi_i_j);
			R_i_j(0,1) = -sin(phi_i_j);
			R_i_j(1,0) =  sin(phi_i_j);
			R_i_j(1,1) =  cos(phi_i_j);

			Vector3d e_i_j(3,1);						// Error function for nodes i and j
			e_i_j.block(0, 0, 2, 1) = R_i_j.transpose()*(R_i.transpose()*(t_j-t_i)-t_i_j);
			e_i_j(2,0) = phi_j-phi_i-phi_i_j;

			// @todo: Check if correct
			Matrix2d dR_i_tans(2,2);					// Derivative of R_i.transpose() by phi_i (32)
			dR_i_tans(0,0) = -sin(phi_i);
			dR_i_tans(0,1) =  cos(phi_i);
			dR_i_tans(1,0) = -cos(phi_i);
			dR_i_tans(1,1) = -sin(phi_i);

			// compute the Jacobians A_i_j and B_i_j of the error function
			Matrix3d A_i_j = Matrix3d::Zero(3,3);		// (32)
			A_i_j.block(0, 0, 2, 2) = -R_i_j.transpose()*R_i.transpose();
			A_i_j.block(0, 2, 2, 1) = R_i_j.transpose()*dR_i_tans*(t_j-t_i);
			A_i_j(2,2) = -1;

			Matrix3d B_i_j = Matrix3d::Zero(3,3);		// (33)
			B_i_j.block(0, 0, 2, 2) = R_i_j.transpose()*R_i.transpose();
			B_i_j(2,2) = 1;

//			std::cout << "A_i_j = \n" << A_i_j << std::endl;
//			std::cout << "B_i_j = \n" << B_i_j << std::endl;

			// compute the contribution of this constraint to the linear system
			Matrix3d omega_i_j = omegas[constraintIndex];
			H.block(i*3, i*3, 3, 3) += A_i_j.transpose()*omega_i_j*A_i_j;
			H.block(i*3, j*3, 3, 3) += A_i_j.transpose()*omega_i_j*B_i_j;
			H.block(j*3, i*3, 3, 3) += B_i_j.transpose()*omega_i_j*A_i_j;
			H.block(j*3, j*3, 3, 3) += B_i_j.transpose()*omega_i_j*B_i_j;

			// compute the coefficient vector
			b.block(i*3, 0, 3, 1) += A_i_j.transpose()*omega_i_j*e_i_j;
			b.block(j*3, 0, 3, 1) += B_i_j.transpose()*omega_i_j*e_i_j;
		}
		// keep the first node fixed
		H.block(0, 0, 3, 3) += Matrix3d::Identity(3, 3);

		// solve the linear system using sparse Cholesky factorization
		VectorXd delta_x = H.ldlt().solve(-b);

		// update the parameters
        for(int i = 0; i<x.size(); i++)
        {
            x[i] += delta_x.block(i*3, 0, 3, 1);
        }
//	}
	// release the first node
	H.block(0, 0, 3, 3) -= Matrix3d::Identity(3, 3);

//	std::cout << "H = \n" << H << std::endl;
//	std::cout << "x = \n" << x << std::endl;

    return x; // @todo: also return H
}

