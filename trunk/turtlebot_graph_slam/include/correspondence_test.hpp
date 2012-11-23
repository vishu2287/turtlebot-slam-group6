#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;


MatrixXd correspondence_test (MatrixXd omega,VectorXd xi, VectorXd mu,MatrixXd Cov_Sigma,int j,int k);
