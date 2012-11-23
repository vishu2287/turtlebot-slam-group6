#include "ros/ros.h"
#include <Eigen/Dense>
#include <graph_init.hpp>
#include <solve.hpp>
#include <correspondence_test.hpp>
#include <reduce.hpp>
using namespace Eigen;

void callback(const ros::TimerEvent&)
{
  MatrixXd u = MatrixXd::Random(2,2);
  //graph_init(u);

  int d = 15;
  MatrixXd omega_tilde = MatrixXd::Random(d,d);
  VectorXd xi_tilde = MatrixXd::Random(d,1);
  MatrixXd omega = MatrixXd::Random(d,d);
  VectorXd xi = MatrixXd::Random(d,1);
   //reduce(omega, xi);
  MatrixXd temp = solve(omega_tilde, xi_tilde, omega, xi);
	std::vector<MatrixXd> testo;
	testo.push_back(temp);
  correspondence_test ( omega, xi, xi,temp,0,1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Timer timer = n.createTimer(ros::Duration(1.0), callback);
  ros::spin();
  return 0;
}


  
