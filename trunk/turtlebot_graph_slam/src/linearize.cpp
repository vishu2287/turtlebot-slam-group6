#include "ros/ros.h"
#include <float.h>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;


//Table 11.2 Page 347/348
//@TODO: Implement
MatrixXd linearize (MatrixXd u, std::vector<MatrixXd> z, std::vector<MatrixXd> c, MatrixXd mu, MatrixXd Xi) {

    // @todo: Check deltaT
    int deltaT = 1;

    int size = 0;
    for (int i = 0; i < z.size(); i++) {
        size += z[i].cols();
    }
    // size of omega and Xi = number of measurements + number of map features
    MatrixXd omega = MatrixXd::Zero(mu.cols() + size, mu.cols() + size);
    Xi = MatrixXd::Zero(mu.cols() + size , 1);
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
        Gt(0, 3) = - (+vt/wt*cos(mu(2,t)) + vt/wt*cos(mu(2,t) + wt*deltaT));
        Gt(1, 3) = (-vt/wt*sin(mu(2,t)) + vt/wt*sin(mu(2,t) + wt*deltaT));

        // Prepare line 7 & 8, create G^T with additional row, value 1
        MatrixXd GtTrans = MatrixXd::Constant(4, 3, 1);
        GtTrans.topLeftCorner(3, 3) = -Gt.transpose();

        // prepare line 7 & 8, create -G with additional column, value 1
        MatrixXd GtMinus = MatrixXd::Constant(3, 4, 1);
        GtMinus.topLeftCorner(3, 3) = Gt;

        // create R^-1, @TODO where to get Rt??
        Matrix3d Rt = Matrix3d::Identity();
        Matrix3d RtInv = Rt.inverse();

        //line 7:
        Matrix4d add1 = GtTrans * RtInv * GtMinus;
        //@TODO: Add to Omega at x(t+1) and x(t)

        //line 8:
        Vector4d add2 = GtTrans * RtInv * (xt - Gt * mu.col(t));
        //@TODO: Add to Xi at x(t+1) and x(t)
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
            // line 14
            Vector2d delta;
            delta << mu(j, 0) - mu(t, 0), mu(j, 1) - mu(t, 1);
            // line 15
            int q = delta(0) * delta(0) + delta(1) * delta(1);
            double sqrtQ = sqrt(q);
            // line 16
            Vector3d Zit;
            // @TODO check for correct indizes especially Sj
            Zit << sqrtQ, atan2(delta(1), delta(2)) - mu(t, 2), z[t](i, j);
            // line 17
            MatrixXd Hit(6, 3);
            Hit.row(0) << -sqrtQ * delta(0), -sqrtQ * delta(1), 0, sqrtQ * delta(0), sqrtQ * delta(1), 0;
            Hit.row(1) << delta(1), -delta(0), -q, -delta(1), delta(0), 0;
            Hit.row(2) << 0, 0, 0, 0, 0, q;
            Hit /= q;
            // Line 18 & 19
            Matrix3d add1 = Hit.transpose() * Qt.inverse() * Hit;
            VectorXd muStar;
            // 6th element is mu(j, s) according to book, but might be a mistake
            // replaced with mu(j, theta)
            muStar << mu(t, 0), mu(t, 1), mu(t, 2), mu(j, 0), mu(j, 1), mu(j, 2);
            Vector3d add2 = Hit.transpose() * Qt.inverse() * (z[t].col(i) - Zit + Hit * muStar);
        }
    }
    return omega;
}
