#include "ros/ros.h"
#include <Eigen/Dense>
#include <graph_init.hpp>
#include <linearize.hpp>
#include <reduce.hpp>
#include <solve.hpp>
#include <correspondence_test.hpp>
using namespace Eigen;

MatrixXd simple_slam (MatrixXd u, std::vector<MatrixXd> z, int deltaT) {
	int t = u.cols();
	// Initialize correspondenses c with a unique value
	std::vector < MatrixXd > c;
	int uniqueValue = 0;
	for (int i = 0; i < t; i++) {
		MatrixXd newCorrespondence(1, z.at(i).cols());
		for (int col = 0; col < newCorrespondence.cols(); col++)
		{
			newCorrespondence(0, col) = uniqueValue;
			uniqueValue++;	// We assume that every observed feature belongs to a different landmark for now
		}
		c.push_back(newCorrespondence);
	}
	// Call initialize method
	MatrixXd muPath = graph_init(u, deltaT);
	// std::cout << "muPath = \n" << muPath << std::endl;
  for(int t = 0; t<u.cols(); t++)
  { 
    for (int i = 0; i < z[t].cols(); i++)
    {
      int j = c[t](0, i);
      // Calculating x and y coordinate of the features and adding them to mu
      if(muPath.cols()/3 < u.cols()+ 1 + j+1) {
//            std::cout << "Adding cols for z["<< t <<"]"<< std::endl;
              MatrixXd newMu(muPath.cols()+z[t].cols()*3, 0);
              newMu.block(0, 0, 1, muPath.cols()) = muPath;
              int index = muPath.cols();
              for (int k = 0; k < z[t].cols(); k++) {

                Vector3d feature = z[t].col(k);
                // x     =    r       * cos(phi        - theta              ) + x of pos
                double x = feature(0) * cos(feature(1) + muPath(3*t+2, 0)) + muPath(3*t, 0);
                // y     =    r       * sin(phi        - theta              ) + y of pos
                double y = feature(0) * sin(feature(1) + muPath(3*t+2, 0)) + muPath(3*t+1, 0);
//                std::cout << "X = " << x << "\n"<<std::endl;
//                std::cout << "Y = " << y << "\n"<<std::endl;
                newMu(index*3, 0) = x;
                newMu(index*3+1, 0) = y;
                newMu(index*3+2, 0) = 1;
                //Only the first and the second element of the feature entries in mu are filled. The orientation is always 0.
                index++;
             }
      muPath = newMu;
      }
    }
  }
  int numberLandmarks = 0;
  for (int i = 0; i < z.size(); i++) {
	  numberLandmarks += z[i].cols();
  }
  VectorXd outMu = VectorXd::Zero(3*(u.cols() + 1 + numberLandmarks), 1);
  for(int i = 0; i < outMu.rows(); i++) {
    int index = (int)(i/3);
    outMu(i) = muPath(0,index);
    outMu(i+1) = muPath(1, index);
    outMu(i+2) = muPath(2, index);
    i+=2;
  } 
	return outMu; // should return full mu
}

