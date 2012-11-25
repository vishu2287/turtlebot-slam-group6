#include "ros/ros.h"
#include <Eigen/Dense>
using namespace Eigen;


double correspondence_test (MatrixXd omega,VectorXd xi, VectorXd mu,MatrixXd Cov_Sigma,int j,int k, int t);
