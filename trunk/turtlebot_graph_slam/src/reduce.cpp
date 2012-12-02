#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;


//Table 11.3 Page 349
//@TODO: Test Properly
std::vector<MatrixXd> reduce (MatrixXd omega,  VectorXd xi) {
	std::vector<MatrixXd> solution;
	int t = 3; // This should be set to the current position of where features start TODO
							
	MatrixXd omega_tilde = omega;			//Deep copy ? //Clone supported in eigen ? TODO
  	MatrixXd xi_tilde = xi;
	const int savei = (t+1)*3;
  	for(int i = (t+1)*3; i < omega.rows(); i+=3){
		std::vector<int> tau;
		//Fill Tau here - i because matrix length changes
		for(int z = 0; z < ((t+1)*3-i) ; z+=3){
			//If any of these values is nonzero, a correspondence between pose and landmark has been found!
			if(omega(i,z)!=0  ||  omega(i,z+1)!=0  ||  omega(i,z+2)!=0
			|| omega(i+1,z)!=0||  omega(i+1,z+1)!=0||  omega(i+1,z+2)!=0
			|| omega(i+2,z)!=0||  omega(i+2,z+1)!=0||  omega(i+2,z+2)!=0)
				//Save the Pose in Tau
				tau.push_back((z/3));
		}
	//subtract (OmegaTilde Tau(j),j) times (Omega j,j inverse) times XI(j) from Xi tilde at Pose Tau(j) and m(j)
	//---------------------------------------------------------------------------------------------------------
		Matrix3d temp_omega_jj_inverse;
		//omega_tilde.block(i,i,3,3) would be the same here

		temp_omega_jj_inverse << omega_tilde(savei,savei), omega_tilde(savei,savei+1), omega_tilde(savei,savei+2),
				    omega_tilde(savei+1,savei), omega_tilde(savei+1,savei+1), omega_tilde(savei+1,savei+2),
			            omega_tilde(savei+2,savei), omega_tilde(savei+2,savei+1), omega_tilde(savei+2,savei+2);
		Matrix3d omega_jj_inverse = temp_omega_jj_inverse.inverse();
		// construct a new Matrix with rows being the poses and columns being j
		MatrixXd omega_tilde_part_left = MatrixXd::Zero(tau.size()*3,3);


		for(int z = 0; z<tau.size()*3;z+=3){
			omega_tilde_part_left.block(z,0,3,3) += omega_tilde.block(tau[z/3],tau[z/3],3,3);
			//THIS should contain all poses in the end
		}
		// OmegaTilde Tau(j),j) times (Omega j,j inverse) times XI(j)
		VectorXd solvingvector = omega_tilde_part_left * omega_jj_inverse * xi.block(i,0,3,1);
		for(int z = 0; z<tau.size()*3;z+=3){
			//Remove from XI TAU(J)
			xi_tilde.block(tau[z/3],0,3,1) -= solvingvector.block(z,0,3,1);
			//REMOVE FROM MJ AT XI TILDE
			xi_tilde.block(savei,0,3,1)   -= solvingvector.block(z,0,3,1);
		}

        //subtract (OmegaTilde Tau(j),j) times (Omega j,j inverse) times Omega(j,Tau(j)) from Omega tilde at Pose Tau(j) and m(j)
	//---------------------------------------------------------------------------------------------------------
		MatrixXd omega_tilde_part_right = MatrixXd::Zero(3,tau.size()*3);

		for(int z = 0; z<tau.size()*3;z+=3){
			omega_tilde_part_right.block(0,z,3,3) += omega_tilde.block(tau[z/3],tau[z/3],3,3);

		}
		MatrixXd solvingmatrix = omega_tilde_part_left * omega_jj_inverse *omega_tilde_part_right ;
		for(int z = 0; z<tau.size()*3;z+=3){
			//Remove from XI TAU(J)
			omega_tilde.block(tau[z/3],tau[z/3],3,3) -= solvingmatrix.block(z,z,3,3);
			//REMOVE FROM MJ AT XI TILDE
			omega_tilde.block(savei,savei,3,3)   -= solvingmatrix.block(z,z,3,3);
		}

	//Remove from Omega and XI All rows and columns corresponding to j !
	//----------------------------------------------------------------------------------------------------------------
		//Vector Removal
		VectorXd final_xi = VectorXd::Zero(xi_tilde.rows()-3);		//Construct final Xi with no Reference to Mj
		final_xi.block(0,0,savei,1)+=xi_tilde.block(0,0,savei,1);

		if((i>=xi_tilde.rows()-3)){
		}
		else{
			final_xi.block(savei,0,xi_tilde.rows()-(savei+3),1)+=xi_tilde.block(savei+3,0,xi_tilde.rows()-(savei+3),1);
		}
		//Removal of mj from final_xi works.

		//Matrix Removal

		MatrixXd final_omega = MatrixXd::Zero(omega_tilde.rows()-3,omega_tilde.cols()-3);
		final_omega.block(0,0,savei,savei)+=omega_tilde.block(0,0,savei,savei);
		if((i>=omega_tilde.cols()-3)){
		}
		else{
		final_omega.block(0,savei,savei,omega_tilde.cols()-(savei+3))+=omega_tilde.block(0,savei+3,savei,omega_tilde.cols()-(savei+3));
		}
		if((savei>=omega_tilde.rows()-3)){
		}
		else{
			final_omega.block(savei,0,omega_tilde.rows()-(savei+3),savei)+=omega_tilde.block(savei+3,0,omega_tilde.rows()-(savei+3),savei);
		}
		if((savei>=omega_tilde.rows()-3) && (i>=omega_tilde.cols()-3)){
		}
		else{
		final_omega.block(savei,savei,omega_tilde.rows()-(savei+3),omega_tilde.cols()-(savei+3))=omega_tilde.block(savei+3,savei+3,omega_tilde.rows()-(savei+3),omega_tilde.cols()-(savei+3));
		}
		//Apply removals :

		omega_tilde = final_omega;
		xi_tilde = final_xi;
	/**/
	} //End of outer loop
		
	solution.push_back(omega_tilde);
	solution.push_back(xi_tilde);
	return solution;
}


	
