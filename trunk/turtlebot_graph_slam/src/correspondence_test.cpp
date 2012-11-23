#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;

//Table 11.4 Page 349
//@TODO: Implement
//TODO Make COV_SIGMA to a list of cov sigma according to time
MatrixXd correspondence_test (MatrixXd omega, VectorXd xi, VectorXd mu,MatrixXd Cov_Sigma,int j,int k) {
	int t = 0; // where features start
	std::vector<int> tau;
	for(int z = 0; z < ((t+1)*3) ; z+=3){		
		//If any of these values is nonzero, a correspondence between EITHER the state and mj, or the state and mk has been found
		if(
		(omega(j,z)!=0  ||  omega(j,z+1)!=0  ||  omega(j,z+2)!=0  
		|| omega(j+1,z)!=0||  omega(j+1,z+1)!=0||  omega(j+1,z+2)!=0
		|| omega(j+2,z)!=0||  omega(j+2,z+1)!=0||  omega(j+2,z+2)!=0)
					||
		(omega(k,z)!=0  ||  omega(k,z+1)!=0  ||  omega(k,z+2)!=0  
		|| omega(k+1,z)!=0||  omega(k+1,z+1)!=0||  omega(k+1,z+2)!=0
		|| omega(k+2,z)!=0||  omega(k+2,z+1)!=0||  omega(k+2,z+2)!=0)
		)
		//Save the Pose in Tau
		tau.push_back((z/3));
	}
	//CONSTRUCT SIGMA Tau(j,k),Tau(j,k) //relatively sure about that one
	MatrixXd sigmapart = MatrixXd::Zero(tau.size()*3,tau.size()*3);
	for(int z = 0; z < tau.size() ; z+=1){
		sigmapart.block(z*3,0,3,3)+= Cov_Sigma.block(tau[z],tau[z],3,3);
	}
	//CONSTRUCT Omegajk,Tau(j,k)
	MatrixXd omega_jk_right = MatrixXd::Zero(6,tau.size()*3);
	for(int z = 0; z < tau.size() ; z+=1){
		//Assumption the jk in Omega jk,Tau(j,k) means we want position j and k resulting in a 6x(Tau(j,k)*3) long matrix
		omega_jk_right.block(0,z*3,3,3)+= omega.block(j,tau[z],3,3);
		omega_jk_right.block(3,z*3,3,3)+= omega.block(k,tau[z],3,3);
	}
	//CONSTRUCT Tau(j,k),Omegajk
	MatrixXd omega_jk_left = MatrixXd::Zero(tau.size()*3,6);
	for(int z = 0; z < tau.size() ; z+=1){
		omega_jk_left.block(z*3,0,3,3)+= omega.block(tau[z],j,3,3);
		omega_jk_left.block(z*3,3,3,3)+= omega.block(tau[z],k,3,3);
	}
	std::cout << "Omega times Sigma times Omega?" << std::endl <<  omega_jk_right * sigmapart * omega_jk_left << std::endl;	//ends in a 6x6 Matrix!

	MatrixXd omega_jk_jk = MatrixXd::Zero(6,6);
	omega_jk_jk.block(0,0,3,3) += omega.block(j,j,3,3);	//Omega jk_jk = Omegaj,j ; Omega j,k ; Omega k,j; Omega k,k ??
	omega_jk_jk.block(0,3,3,3) += omega.block(j,k,3,3);
	omega_jk_jk.block(3,0,3,3) += omega.block(k,j,3,3);
	omega_jk_jk.block(3,3,3,3) += omega.block(k,k,3,3);

	//Assumption : Omega[j,k] = 6x6 matrix ...
	MatrixXd omega_j_k = MatrixXd::Zero(6,6);
	omega_j_k = omega_jk_jk - omega_jk_right * sigmapart * omega_jk_left;
	std::cout << "Omega j_k" << std::endl << omega_j_k << std::endl;
	//Construct mu j,k as seen on page 367
	MatrixXd omega_jk_jk_inverse = MatrixXd::Zero(6,6);
	omega_jk_jk_inverse = omega_jk_jk.inverse();
	VectorXd mu_taujk = VectorXd::Zero(tau.size()*3);
	for(int z = 0; z < tau.size() ; z+=1){
		//mu_taujk.block(z*3,1,3,1) += mu.block(tau[z],1,3,1);		// TODO mu_taujk sinnvoll füllen
	}
	std::cout << "Test" << std::endl << mu_taujk << std::endl;
	return omega;

}
