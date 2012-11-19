#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;


//Table 11.3 Page 349
//@TODO: Implement
std::vector<MatrixXd> reduce (MatrixXd omega,  VectorXd xi) {
	std::vector<MatrixXd> solution;
	int t = 1; // This should be set to the current position of m !
							
	MatrixXd omega_tilde = omega;			//Deep copy ? //Clone supported in eigen ?
  	VectorXd xi_tilde = xi;
	std::cout << "Omega Matrix" << std::endl <<  omega << std::endl;
  	for(int i = (t+1)*3; i < omega.rows(); i+=6){
		std::vector<int> tau;
		//Fill Tau here
		for(int z = 0; z < (t+1)*3 ; z+=3){		
			//If any of these values is nonzero, a correspondence between pose and landmark has been found!	
			if(omega(i,z)!=0  ||  omega(i,z+1)!=0  ||  omega(i,z+2)!=0  
			|| omega(i+1,z)!=0||  omega(i+1,z+1)!=0||  omega(i+1,z+2)!=0
			|| omega(i+2,z)!=0||  omega(i+2,z+1)!=0||  omega(i+2,z+2)!=0)
				//Save the Pose in Tau
				tau.push_back((z/3));
				std::cout << "Omega Matrix" << std::endl <<  (z/3) << std::endl;
		}
	//subtract (OmegaTilde Tau(j),j) times (Omega j,j inverse) times XI(j) from Xi tilde at Pose Tau(j) and m(j)
	//---------------------------------------------------------------------------------------------------------
		//Do you agree to this structure ?
		Matrix3d temp_omega_jj_inverse;
		temp_omega_jj_inverse << omega_tilde(i,i), omega_tilde(i,i+1), omega_tilde(i,i+2),
				    omega_tilde(i+1,i), omega_tilde(i+1,i+1), omega_tilde(i+1,i+2),
			            omega_tilde(i+2,i), omega_tilde(i+2,i+1), omega_tilde(i+2,i+2);
		//omega_tilde.block(i,i,3,3) 
		Matrix3d omega_jj_inverse = temp_omega_jj_inverse.inverse();	
		// construct a new Matrix with rows being the poses and columns being j
		MatrixXd omega_tilde_part = MatrixXd::Zero(tau.size()*3,3);
				std::cout << "Temp Omega JJ" << std::endl <<  omega_tilde_part << std::endl;
		for(int z = 0; z<tau.size()*3;z+=3){
			omega_tilde_part.block(z,0,3,3) += omega_tilde.block(tau[z/3],tau[z/3],3,3);
			//THIS should contain all poses in the end
		}

		std::cout << "xi_tilde before apllying solution" << std::endl <<  xi_tilde << std::endl;
		// OmegaTilde Tau(j),j) times (Omega j,j inverse) times XI(j)
		VectorXd solvingvector = omega_tilde_part * omega_jj_inverse * xi.block(i,0,3,1);
		for(int z = 0; z<tau.size()*3;z+=3){
			//Remove from XI TAU(J)
			xi_tilde.block(tau[z/3],0,3,1) -= solvingvector.block(z,0,3,1);
			//REMOVE FROM MJ AT XI TILDE
			xi_tilde.block(i,0,3,1)   -= solvingvector.block(z,0,3,1);
		}


		std::cout << "xi_tilde after apllying solution" << std::endl <<  xi_tilde << std::endl;
	
        //subtract (OmegaTilde Tau(j),j) times (Omega j,j inverse) times Omega(j,Tau(j)) from Omega tilde at Pose Tau(j) and m(j)
	//---------------------------------------------------------------------------------------------------------	
		
	}
	return solution;
}


	
