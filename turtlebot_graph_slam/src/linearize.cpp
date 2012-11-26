#include "ros/ros.h"
#include <float.h>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;


//Table 11.2 Page 347/348
MatrixXd linearize (MatrixXd u, std::vector<MatrixXd> z, std::vector<MatrixXd> c, MatrixXd mu, int deltaT) {

   int numberLandmarks = 0;
   for (int i = 0; i < z.size(); i++) {
	   numberLandmarks += z[i].cols();
   }
   // size of omega and Xi = number of measurements + number of map features
   MatrixXd omega = MatrixXd::Zero(3*(mu.cols() + numberLandmarks), 3*(mu.cols() + numberLandmarks));
   VectorXd xi = VectorXd::Zero(3*(mu.cols() + numberLandmarks), 1);
   Matrix3d inf = Matrix3d::Identity() * DBL_MAX;
   omega.topLeftCorner(3, 3) = inf;
   Vector3d xt;

   // line 4, start first loop
   // t is shifted by one, so t = t-1 according to the algorithm
   for(int t = 0; t<u.cols(); t++)
   {
       // line 5, init xt
       double vt = u(0,t);
       double wt = u(1,t);
       xt(0) = mu(0, t) + (-vt/wt*sin(mu(2,t)) + vt/wt*sin(mu(2,t) + wt*deltaT));
       xt(1) = mu(1, t) + (+vt/wt*cos(mu(2,t)) - vt/wt*cos(mu(2,t) + wt*deltaT));
       xt(2) = mu(2, t) + (wt*deltaT);

       // line 6, init Gt
       Matrix3d Gt = Matrix3d::Identity();
       Gt(0, 2) = - (+vt/wt*cos(mu(2,t)) + vt/wt*cos(mu(2,t) + wt*deltaT));
       Gt(1, 2) = (-vt/wt*sin(mu(2,t)) + vt/wt*sin(mu(2,t) + wt*deltaT));

       // Prepare line 7 & 8, create G^T with additional row, value 1
       MatrixXd GtTrans = MatrixXd::Constant(6, 3, 1);
       GtTrans.topLeftCorner(3, 3) = -Gt.transpose();
       GtTrans.bottomLeftCorner(3, 3) = Matrix3d::Identity();

       // prepare line 7 & 8, create -G with additional column, value 1
       MatrixXd GtMinus = MatrixXd::Constant(3, 6, 1);
       GtMinus.topLeftCorner(3, 3) = Gt;
       GtMinus.bottomLeftCorner(3, 3) = Matrix3d::Identity();

       // create R^-1, @TODO where to get Rt??
       Matrix3d Rt = Matrix3d::Identity();
       Matrix3d RtInv = Rt.inverse();

       //line 7:
       // int xt = t*3;
       int xt1 = t*3;
       MatrixXd add1 = GtTrans * RtInv * GtMinus; // 6x6
       omega.block(xt1, xt1, 6, 6) += add1;

       //@TODO: Add to Omega at x(t+1) and x(t). just in case
       // Matrix3d xtxt = add1.bottomRightCorner(3, 3);
       // Matrix3d xt1xt = add1..topRightCorner(3, 3);
       // Matrix3d xtxt1 = add1..bottomLeftCorner(3, 3);
       // Matrix3d xt1xt1 = add1..topLeftCorner(3, 3);

      // omega.block(xt, xt, 3, 3)+=xtxt;
      // omega.block(xt, xt1, 3, 3)+=xtxt1;
      // omega.block(xt1, xt, 3, 3)+=xt1xt;
      // omega.block(xt1, xt1, 3, 3)+=xt1xt1;

       //line 8:
       VectorXd add2 = GtTrans * RtInv * (xt - Gt * mu.col(t));
       //@TODO: Add to Xi at x(t+1) and x(t)
       xi.block(xt1, 0, 6, 1) += add2;

   } // end loop

   for(int t = 0 ; t < z.size() ; t++)
   {
       // line 11, calculate sigma and create Qt
       Matrix3d Qt = Matrix3d::Zero();

       //Variances
       double rSum = 0;
       double phiSum = 0;
       double sSum = 0;
       for (int i = 0; i < z[t].cols(); i++)
       {
           rSum += z[t](0, i);
           phiSum += z[t](1, i);
           sSum += z[t](2, i);
       }
       double muR = rSum / z[t].cols();
       double muPhi = phiSum / z[t].cols();
       double muS = sSum / z[t].cols();
       double sigSqR = 0;
       double sigSqPhi = 0;
       double sigSqS = 0;
       for (int i = 0; i < z[t].cols(); i++)
       {
           sigSqR+= pow(z[t](0, i), 2) - pow(muR, 2);
           sigSqPhi  += pow(z[t](1, i), 2) - pow(muPhi, 2);
           sigSqS += pow(z[t](2, i), 2) - pow(muS, 2);
       }
       Qt(0,0) = sigSqR;
       Qt(1,1) = sigSqPhi;
       Qt(2,2) = sigSqS;

       // line 12, inner loop over each matrix, extracting the features
       for (int i = 0; i < z[t].cols(); i++)
       {
           // line 13
           // each c[t] as one row, like a transposed vector
           int j = c[t](0, i);

           /*
            * THIS IS A CRITICAL STEP:
            * HERE WE HAVE TO CHECK IF MU ALREADY HAS AN ENTRY FOR LANDMARK J.
            * IF NOT - WHICH IS THE CASE EVERY TIME RIGHT NOW - WE HAVE TO PUT A NEW ENTRY INTO MU
            * AT (t+1+j) WITH THE X AND Y POSITION OF THE LANDMARK. ONLY THEN CAN WE MOVE ON WITH
            * THE REST OF THE CODE, THATS WHY IT IS COMMENTED FOR NOW.
            */

//           // line 14
//           Vector2d delta;
//           delta << mu(j, 0) - mu(t, 0), mu(j, 1) - mu(t, 1);
//           // line 15
//           int q = delta(0) * delta(0) + delta(1) * delta(1);
//           double sqrtQ = sqrt(q);
//           // line 16
//           Vector3d Zit;
//           // @TODO check for correct indizes especially Sj
//           // @TODO here is a bug that need to be fixed, seems to be at least one of the indizes
//           std::cout << "start" << std::endl;
//           std::cout << "t=" << t << "; i=" << i << "; j=" << j << std::endl;
//           std::cout << "mu: " << mu.cols() << "; " << mu.rows() << std::endl;
//           std::cout << "z[t]: " << z[t].cols() << "; " << z[t].rows() << std::endl;
//           Zit << sqrtQ, atan2(delta(1), delta(0)) - mu(t, 2), z[t](i, j);
//           std::cout << "end" << std::endl;
//           // line 17
//           MatrixXd Hit(6, 3);
//           Hit.row(0) << -sqrtQ * delta(0), -sqrtQ * delta(1), 0, sqrtQ * delta(0), sqrtQ * delta(1), 0;
//           Hit.row(1) << delta(1), -delta(0), -q, -delta(1), delta(0), 0;
//           Hit.row(2) << 0, 0, 0, 0, 0, q;
//           Hit /= q;
//           std::cout << "Line 18 start" << std::endl;
//           // Line 18 & 19
//           MatrixXd add1 = Hit.transpose() * Qt.inverse() * Hit;
//           VectorXd muStar;
//
//
//          int xt = t*3;
//          int mj = mu.cols()*3+j*3;
//          Matrix3d xtxt = add1.topLeftCorner(3, 3);
//          Matrix3d xtmj = add1.topRightCorner(3, 3);
//          Matrix3d mjxt = add1.bottomLeftCorner(3, 3);
//          Matrix3d mjmj = add1.bottomRightCorner(3, 3);
//
//          omega.block(xt, xt, 3, 3)+=xtxt;
//          omega.block(xt, mj, 3, 3)+=xtmj;
//          omega.block(mj, xt, 3, 3)+=mjxt;
//          omega.block(mj,mj, 3, 3)+=mjmj;
//          // @Todo;6th element is mu(j, s) according to book, but might be a mistake
//          // replaced with mu(j, theta)
//          muStar << mu(t, 0), mu(t, 1), mu(t, 2), mu(j, 0), mu(j, 1), mu(j, 2);
//          VectorXd add2 = Hit.transpose() * Qt.inverse() * (z[t].col(i) - Zit + Hit * muStar);
//          Vector3d xtT = add2.segment(0,3);
//          Vector3d xtB = add2.segment(3,3);
//          xi.block(xt, 0, 3, 0)+=xtT;
//          xi.block(mj, 0, 3, 0)+= xtB;

       }
   }
   return omega;
}

