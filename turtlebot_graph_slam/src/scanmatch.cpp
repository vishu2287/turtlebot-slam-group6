#include "ros/ros.h"
#include <cstdlib>
#include <ctime>
#include <queue>
#include <map>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "nav_msgs/OccupancyGrid.h"
#include <geometry_msgs/Point32.h>
using namespace Eigen;
//TODO FIX LASERSCAN SIZE so both scans have the same number of scans

double getDistance(geometry_msgs::Point32 one, geometry_msgs::Point32 two) {
    return sqrt(pow((one.x - two.x), 2) + pow((one.y - two.y), 2));
}

MatrixXd scanmatch(sensor_msgs::PointCloud model ,  sensor_msgs::PointCloud target){
    MatrixXd robotpos = MatrixXd::Zero(3,1);
    ROS_INFO_STREAM("Scanmatcher is starting...");
    //while ! converging  			//START of the icp iterations
    int modelSize = model.points.size();
    int saver[modelSize];
    int its = 0;
    int maxIts = 500;
    bool errors[modelSize];
    double delta = 0.02;
    double distance = 0;
    double oldDistance = 1000;
    double robotx = 0;
    double roboty = 0;
    double robotphi = 0;
    double MAX_DISTANCE = 0.5;
    double distanceDiff = 100;
    double centroidTargetX = 0;
    double centroidTargetY = 0;
    double centroidModelX = 0;
    double centroidModelY = 0;
    double modelX = 0;
    double modelY = 0;
    double targetX = 0;
    double targetY = 0;
    double distances[modelSize];
    MatrixXd Mtd;
    MatrixXd Mmd;
    Matrix3d Tmat;
    MatrixXd H;
    Matrix2d U;
    Matrix2d V;
    Matrix2d R;
    Vector2d centerOfModel;
    Vector2d centerOfTarget;
    Vector2d t;
    Vector3d temp_coord;
    //for(int runthroughs = 0; runthroughs < Threshold ; runthroughs++){
    while(distanceDiff > delta && its <= maxIts) {
        its++;
        oldDistance = distance;
        distance = 0;
        int numberofpairs = 0;
        // SEARCH CLOSEST NEIGHBOR
        //-----------------------------------------------------------------------
        for(int i = 0; i < model.points.size();i++){
            double prev = 1000000;
            for(int z = 0; z < target.points.size();z++){
                double length = getDistance(target.points[z], model.points[i]);
                if(length < prev){
                    if(length >= MAX_DISTANCE){
                        errors[i] = true;

                    } else {
                        //Save position of closest neighbor here
                        saver[i] = z;
                        distances[i] = length;

                        errors[i] = false;
                    }
                    prev = length;
                }
            }
        }
        for(int i = 0; i < modelSize; i++) {
            if(!errors[i]) {
                numberofpairs++;
                distance += distances[i];
            }
        }
//        std::cout << "Total distance = " << distance << "\n";
        // Calculate center points of both scans
        // ----------------------------------------------------
        centroidTargetX = 0;
        centroidTargetY = 0;
        centroidModelX = 0;
        centroidModelY = 0;
        modelX = 0;
        modelY = 0;
        targetX = 0;
        targetY = 0;
        for(int i = 0; i < model.points.size();i++){
            if(!errors[i]){
                modelX+= model.points[i].x;
                modelY+= model.points[i].y;
                targetX+= target.points[saver[i]].x;
                targetY+= target.points[saver[i]].y;
            }
        }
        centroidModelX = modelX/numberofpairs;			//Checken ob es double bleibt
        centroidModelY = modelY/numberofpairs;
        centroidTargetX = targetX/numberofpairs;
        centroidTargetY = targetY/numberofpairs;
//        ROS_INFO_STREAM("Correspondence and centroids created !");
        // Calculate Correlation matrix H
        // ----------------------------------------------------
        Mtd = MatrixXd::Zero(2,numberofpairs);
        Mmd = MatrixXd::Zero(2,numberofpairs);
        int counter = 0;
        for(int i = 0; i < model.points.size();i++){
            if(!errors[i]){
                Mmd(0,i-counter) = model.points[i].x-centroidModelX;
                Mmd(1,i-counter) = model.points[i].y-centroidModelY;
                Mtd(0,i-counter) = target.points[saver[i]].x-centroidTargetX;
                Mtd(1,i-counter) = target.points[saver[i]].y-centroidTargetY;
            } else {
                counter++;
            }
        }
        H = Mmd*Mtd.transpose() ;
//        ROS_INFO_STREAM("Matrizes created!");
        // Apply SVD (Singular value decomposition) TO GET rOTATION mATRIX r
        // ----------------------------------------------------
        JacobiSVD<MatrixXd> svd(H,Eigen::ComputeThinU | Eigen::ComputeThinV);//(H);//SVD<MatrixXd> svdOfA(H);
        U = svd.matrixU();
        V = svd.matrixV();
        R = V*U.transpose();
        //std::cout << "R" << R;
        // Estimate translation vector and build final transformation matrix
        // ----------------------------------------------------
//        std::cout << "R" << R;
//         ROS_INFO_STREAM("SVD complete");
        centerOfModel(0) = centroidModelX;
        centerOfModel(1) = centroidModelY;
        centerOfTarget(0) = centroidTargetX;
        centerOfTarget(1) = centroidTargetY;
        t = centerOfTarget - R * centerOfModel;
        Tmat << R(0,0),R(0,1),t(0),
                R(1,0),R(1,1),t(1),
                0,       0,     1;
//        std::cout << "Translation Matrix" << R;
//        ROS_INFO_STREAM("Tmat created!");
        // Final calculation
        // ----------------------------------------------------
        for(int i = 0; i < model.points.size();i++){
            Vector3d M;
            M(0) = model.points[i].x;
            M(1) = model.points[i].y;
            M(2) = 1;
            Vector3d sol = Tmat * M;
            model.points[i].x = sol(0);
            model.points[i].y = sol(1);
        }

        temp_coord(0) = robotx;
        temp_coord(1) = roboty;
        temp_coord(2) = 1;

        Vector3d robotsol = Tmat * temp_coord;

        robotx = robotsol(0);
        roboty = robotsol(1);
        robotphi += acos(R(0,0));

        distanceDiff = (oldDistance - distance);
//        std::cout << "distanceDiff :" << distanceDiff << "\n";
        if(distanceDiff < 0) {
            distanceDiff *= -1;
//            ROS_ERROR("DISTANCE INCREASED!");
        }
//        std::cout << "distanceDiff :" << distanceDiff << "\n";
    }
    std::cout << "\nScan matcher took " << its << " iterations.\n";
    robotpos(0,0) = robotx;
    robotpos(1,0) = roboty;
    robotpos(2,0) = robotphi;
    return robotpos;
}
/*bool contains(int test [], int i){
    for(int i= 0; i < sizeof(test) ; i++){
        if(test[i] == i)
            return true;
    }
    return false;
}
sensor_msgs::PointCloud scanmatch(sensor_msgs::PointCloud one ,  sensor_msgs::PointCloud two){
 ROS_INFO_STREAM("Scanmatcher is starting...");
    //while ! converging  			//START of the icp iterations
    int secondsize = two.points.size();
    int saver[two.points.size()];
    double lengthsaver[two.points.size()];
    int Threshold = 5;
    std::vector <geometry_msgs::Point32 > firstpoints;
    std::vector <geometry_msgs::Point32 > secondpoints;
    for(int runthroughs = 0; runthroughs < Threshold ; runthroughs++){
        double sum = 0;
         // SEARCH CLOSEST NEIGHBOR
         //-----------------------------------------------------------------------
         for(int i = 0; i < two.points.size();i++){
            double prev = 1000000;
            for(int z = 0; z < one.points.size();z++){
                double length = sqrt(pow(one.points[z].x-two.points[i].x,2)+pow(one.points[z].y-two.points[i].y,2));
                if(length == 0)
                    continue;
                if(length < prev){
                    //First occurence of point pair
                    if(lengthsaver[i] == 0){
                         //Save position of closest neighbor here
                         lengthsaver[i] = length;
                         saver[i] = z;
                    }
                    //If pair already exists
                    else if(contains(saver,z)){
                        if(lengthsaver[i] < length){
                        }
                        // If save length > current length
                        else{
                            lengthsaver[i] = length;
                                    saver[i] = z;
                        }
                    } else {
                        lengthsaver[i] = length;
                        saver[i] = z;
                    }
                    prev = length;
                }
            }
         }
        for(int i = 0; i < two.points.size();i++){

        }*/
