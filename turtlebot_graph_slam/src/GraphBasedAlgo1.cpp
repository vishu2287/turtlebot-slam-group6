#include "ros/ros.h"
#include <float.h>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

MatrixXd algorithm1(VectorXd x, std::vector<Vector3d> z, std::vector<Matrix3d> omega, std::vector<int> is, std::vector<int> js) {
	// find the maximum likelihood solution
	bool converged = false;

	VectorXd b(x.rows(),1);
	MatrixXd H(x.rows(),x.rows());

	while(!converged)
	{
		VectorXd b = VectorXd::Zero(x.rows(),1);
		MatrixXd H = MatrixXd::Zero(x.rows(),x.rows());
		for(int constraintIndex = 0; constraintIndex < z.size(); constraintIndex++)
		{
			// Get the indices of the nodes this constraint refers to
			int i = is[constraintIndex];
			int j = js[constraintIndex];

			Vector3d x_i = x.block(i, 0, 3, 1);			// (28)
			Vector3d x_j = x.block(j, 0, 3, 1);
			Vector3d z_i_j = z[constraintIndex];		// (29)

			Vector2d t_i = x_i.block(0, 0, 2, 1);		// Position vector of node i
			Vector2d t_j = x_j.block(0, 0, 2, 1);		// Position vector of node j
			Vector2d t_i_j = z_i_j.block(0, 0, 2, 1);	// Measured position difference vector between nodes i and j

			double phi_i = x_i(2,0);					// Angle of node i
			double phi_j = x_j(2,0);					// Angle of node j
			double phi_i_j = z_i_j(2,0);				// Measured angle between nodes i and j

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

			// compute the contribution of this constraint to the linear system
			Matrix3d omega_i_j = omega[constraintIndex];
			H.block(i, i, 3, 3) += A_i_j.transpose()*omega_i_j*A_i_j;
			H.block(i, j, 3, 3) += A_i_j.transpose()*omega_i_j*B_i_j;
			H.block(j, i, 3, 3) += B_i_j.transpose()*omega_i_j*A_i_j;
			H.block(j, j, 3, 3) += B_i_j.transpose()*omega_i_j*B_i_j;

			// compute the coefficient vector
			b.block(i, 0, 3, 1) += A_i_j.transpose()*omega_i_j*e_i_j;
			b.block(j, 0, 3, 1) += B_i_j.transpose()*omega_i_j*e_i_j;
		}
		// keep the first node fixed
		H.block(0, 0, 3, 3) += Matrix3d::Identity(3, 3);

		// solve the linear system using sparse Cholesky factorization
		Vector3d delta_x = H.ldlt().solve(b);

		// update the parameters
		x += delta_x;
	}
	// release the first node
	H.block(0, 0, 3, 3) -= Matrix3d::Identity(3, 3);

	return H; // @todo: also return x
}

