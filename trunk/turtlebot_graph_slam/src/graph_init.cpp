#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;


//Table 11.1 Page 347
//@TODO: Implement
MatrixXd graph_init (MatrixXd u) {

	MatrixXd mu = MatrixXd::Random(2,2);
  	
	//ROS_INFO(mu);
	std::cout << "Here is the matrix u:\n" << u << std::endl;
	std::cout << "Here is the matrix mu:\n" << mu << std::endl;
	std::cout << "u*mu = :\n" << u*mu << std::endl;
	return u;
}


	
