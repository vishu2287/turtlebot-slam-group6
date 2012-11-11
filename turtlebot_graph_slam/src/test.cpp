#include "ros/ros.h"
#include <Eigen/Dense>
#include <graph_init.hpp>
using namespace Eigen;

void callback(const ros::TimerEvent&)
{
  MatrixXd u = MatrixXd::Random(2,2);
  graph_init(u);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Timer timer = n.createTimer(ros::Duration(1.0), callback);
  ros::spin();
  return 0;
}


  
