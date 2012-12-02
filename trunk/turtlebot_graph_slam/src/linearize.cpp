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

   /**
    * NUMBER OF COLUMNS OF OMEGA IS EXTENDED BY 1 TO MAKE SPACE FOR XI, WHICH IS ADDED AT THE END
    */
   MatrixXd omegaAndXi = MatrixXd::Zero(3*(u.cols() + 1 + numberLandmarks), 3*(u.cols() + 1 + numberLandmarks)+1);
   VectorXd xi = VectorXd::Zero(3*(u.cols() + 1 + numberLandmarks), 1);
   Matrix3d inf = Matrix3d::Identity() * DBL_MAX;
   omegaAndXi.topLeftCorner(3, 3) = inf;
   Vector3d xt;

   // line 4, start first loop
   // t is shifted by one, so t = t-1 according to the algorithm
   for(int t = 0; t<u.cols(); t++)
    {
        // line 5, init xt
        double vt = u(0, t);
        double wt = u(1, t);
        xt(0) = mu(0, t) + (-vt/wt*sin(mu(2,t)) + vt/wt*sin(mu(2,t) + wt*deltaT));
        xt(1) = mu(1, t) + (+vt/wt*cos(mu(2,t)) - vt/wt*cos(mu(2,t) + wt*deltaT));
        xt(2) = mu(2, t) + (wt*deltaT);

        // line 6, init Gt
        Matrix3d Gt = Matrix3d::Identity();
        Gt(0, 2) = - (+vt/wt*cos(mu(2,t)) + vt/wt*cos(mu(2,t) + wt*deltaT));
        Gt(1, 2) = (-vt/wt*sin(mu(2,t)) + vt/wt*sin(mu(2,t) + wt*deltaT));

        // Prepare line 7 & 8, create G^T with additional row, value 1
        MatrixXd GtTrans(6, 3);
        GtTrans.topLeftCorner(3, 3) = -Gt.transpose();
        GtTrans.bottomLeftCorner(3, 3) = Matrix3d::Identity();

        // prepare line 7 & 8, create -G with additional column, value 1
        MatrixXd GtMinus(3, 6);
        GtMinus.topLeftCorner(3, 3) = -Gt;
        GtMinus.topRightCorner(3, 3) = Matrix3d::Identity();

        // create R^-1, @TODO where to get Rt??
        Matrix3d Rt = Matrix3d::Identity();
        Matrix3d RtInv = Rt.inverse();

        //line 7:
        // int xt = t*3;
        int xt1 = t*3;
        MatrixXd add1 = GtTrans * RtInv * GtMinus; // 6x6
        omegaAndXi.block(xt1, xt1, 6, 6) += add1;

        //line 8:
        VectorXd add2 = GtTrans * RtInv * (xt - Gt * mu.col(t));
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

//       std::cout << "z = \n" << z.at(t) << std::endl;
       // line 12, inner loop over each matrix, extracting the features
       for (int i = 0; i < z[t].cols(); i++)
       {
           // line 13
           // each c[t] as one row, like a transposed vector
           // mu has u.cols() + 1 columns of robot poses. 
           int j = c[t](0, i);

           //Calculating x and y coordinate of the features and adding them to mu
           if(mu.cols() < u.cols()+ 1 + j+1) { 
//            std::cout << "Adding cols for z["<< t <<"]"<< std::endl;
              MatrixXd newMu(3, mu.cols() + z[t].cols());
              newMu.block(0, 0, 3, mu.cols()) = mu;
              int index = mu.cols();
              for (int k = 0; k < z[t].cols(); k++) {

                Vector3d feature = z[t].col(k);
                // x     =    r       * cos(phi        - theta      ) + x of pos
                double x = feature(0) * cos(feature(1) - mu(2, t)) + mu(0, t);
                // y     =    r       * sin(phi        - theta      ) + y of pos
                double y = feature(0) * sin(feature(1) - mu(2, t)) + mu(1, t);
                newMu(0, index) = x;
                newMu(1, index) = y;
                newMu(2, index) = 1;
                //Only the first and the second element of the feature entries in mu are filled. The orientation is always 0.
                index++;
             }   
             mu = newMu;  
           }
//           std::cout << "THE mu = \n" << mu << std::endl;

           /*
            * THIS IS A CRITICAL STEP:
            * HERE WE HAVE TO CHECK IF MU ALREADY HAS AN ENTRY FOR LANDMARK J.
            * IF NOT - WHICH IS THE CASE EVERY TIME RIGHT NOW - WE HAVE TO PUT A NEW ENTRY INTO MU
            * AT (t+1+j) WITH THE X AND Y POSITION OF THE LANDMARK. ONLY THEN CAN WE MOVE ON WITH
            * THE REST OF THE CODE, THATS WHY IT IS COMMENTED FOR NOW.
            */

           // line 14
           Vector2d delta;
           delta << mu(0, u.cols() + 1 + j) - mu(0, t), mu(1, u.cols() + 1 + j) - mu(1, t);
//            std::cout << "delta = \n" << delta << std::endl;
          
           // line 15
           double q = delta(0) * delta(0) + delta(1) * delta(1);
           if(q == 0)
        	   q = 0.000000001;
//            std::cout << "q =  \n" << q << std::endl;
           double sqrtQ = sqrt(q);
           // line 16
           Vector3d Zit;
           // @TODO check for correct indizes especially Sj <--
           // @TODO here is a bug that need to be fixed, seems to be at least one of the indizes
//           std::cout << "start" << std::endl;
//           std::cout << "t=" << t << "; i=" << i << "; j=" << j << std::endl;
//           std::cout << "mu: " << mu.cols() << "; " << mu.rows() << std::endl;
//           std::cout << "z[t]: " << z[t].cols() << "; " << z[t].rows() << std::endl;
           Zit << sqrtQ, atan2(delta(1), delta(0)) - mu(2, t), mu(2, u.cols() + 1+ j); //<-- Last element = Sj
          // std::cout << "Zit =  \n" << Zit << std::endl;
//           std::cout << "end" << std::endl;
           // line 17
           MatrixXd Hit(3, 6);
           Hit.row(0) << -sqrtQ * delta(0), -sqrtQ * delta(1), 0, sqrtQ * delta(0), sqrtQ * delta(1), 0; // + signs in the book page 348 line 17
           Hit.row(1) << delta(1), -delta(0), -q, -delta(1), delta(0), 0;
           Hit.row(2) << 0, 0, 0, 0, 0, q;
           Hit /= q;
          // std::cout << "Hit =  \n" << Hit << std::endl;
          // std::cout << "Qt inv =  \n" << Qt.inverse() << std::endl;
//           std::cout << "Line 18 start" << std::endl;
           // Line 18 & 19  
           MatrixXd add1 = Hit.transpose() * Qt.inverse() * Hit;
           VectorXd muStar(6);

//           std::cout << "This will be added to omega =  \n" << add1 << std::endl;

           int xt = t*3;
           int mj = (u.cols()+1)*3+j*3;
           Matrix3d xtxt = add1.topLeftCorner(3, 3);
           Matrix3d xtmj = add1.topRightCorner(3, 3);
           Matrix3d mjxt = add1.bottomLeftCorner(3, 3);
           Matrix3d mjmj = add1.bottomRightCorner(3, 3);

           omegaAndXi.block(xt, xt, 3, 3)+=xtxt;
           omegaAndXi.block(xt, mj, 3, 3)+=xtmj;
           omegaAndXi.block(mj, xt, 3, 3)+=mjxt;
           omegaAndXi.block(mj,mj, 3, 3)+=mjmj;
           // std::cout << "omega =  \n" << omega << std::endl; // not a good idea apparently...
           // @Todo;6th element is mu(j, s) according to book, but might be a mistake
           // replaced with mu(j, theta) which is always 0 right now!
           muStar << mu(0, t), mu(1, t), mu(2, t), mu(0, u.cols() + 1 + j), mu(1, u.cols() + 1 + j), mu(2, u.cols() + 1 + j);
//            std::cout << "the big mu vector  =  \n" << muStar << std::endl;
//           std::cout << "Hit.transpose() =  \n" << Hit.transpose() << std::endl;
//           std::cout << "Qt.inverse() =  \n" << Qt.inverse() << std::endl;
//           std::cout << "z[t].col(i) =  \n" << z[t].col(i) << std::endl;
//           std::cout << "Zit =  \n" << Zit << std::endl;
//           std::cout << "Hit =  \n" << Hit << std::endl;
//           std::cout << "muStar =  \n" << muStar << std::endl;
           VectorXd add2 = Hit.transpose() * Qt.inverse() * (z[t].col(i) - Zit + Hit * muStar);
//            std::cout << "This will be added to xi at "<< xt << " - " << mj << " =  \n" << add2 << std::endl;

           Vector3d xtT = add2.segment(0,3);
           Vector3d xtB = add2.segment(3,3);

           xi.block(xt, 0, 3, 1)+=xtT;
           xi.block(mj, 0, 3, 1)+= xtB;

       }
   }
   // Add xi to omegaAndXi
   omegaAndXi.topRightCorner(xi.rows(),1) = xi;
   return omegaAndXi;
}

