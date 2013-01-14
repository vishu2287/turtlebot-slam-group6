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

double CLOSEST_NEIGHBOR_RANGE = 0.1;

double getDistance(double x1,double x2, double y1, double y2){
    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

Vector3d scanmatch(sensor_msgs::PointCloud prev ,  sensor_msgs::PointCloud current){

    std::vector<geometry_msgs::Point32> modelDataSet;
    std::vector<geometry_msgs::Point32> targetDataSet;

    /**
      Get closest neighbor from model to target
    */
    for(int i = 0; i<prev.points.size(); i++)
    {
        double x1Model = prev.points[i].x;
        double y1Model = prev.points[i].y;

        double minDistanceModel = 1000;
        int closestModel = -1;

        for(int j = 0; j<current.points.size(); j++)
        {
            double x2Model = current.points[j].x;
            double y2Model = current.points[j].y;
            double distModel = getDistance(x1Model, x2Model, y1Model, y2Model);
            if(distModel < minDistanceModel)
            {
                minDistanceModel = distModel;
                closestModel = j;
            }
        }

        if(minDistanceModel < CLOSEST_NEIGHBOR_RANGE)
        {
            modelDataSet.push_back(prev.points[i]);
            targetDataSet.push_back(current.points[closestModel]);
        }
    }

    // Initialise transformation correction vector
    Vector3d correction = VectorXd::Zero(3,1);

    // Only if closest neighbors have been found
    if(!modelDataSet.empty())
    {
        /**
        Calculate centroids
        */
        // Model
        double centroidModelX = 0;
        double centroidModelY = 0;
        for(int i = 0; i<modelDataSet.size(); i++)
        {
            centroidModelX+=modelDataSet[i].x;
            centroidModelY+=modelDataSet[i].y;
        }
        centroidModelX /= modelDataSet.size();
        centroidModelY /= modelDataSet.size();

        // Target
        double centroidTargetX = 0;
        double centroidTargetY = 0;
        for(int i = 0; i<targetDataSet.size(); i++)
        {
            centroidTargetX+=targetDataSet[i].x;
            centroidTargetY+=targetDataSet[i].y;
        }
        centroidTargetX /= targetDataSet.size();
        centroidTargetY /= targetDataSet.size();

        Vector2d centroidModel;
        centroidModel(0) = centroidModelX;
        centroidModel(1) = centroidModelY;
        Vector2d centroidTarget;
        centroidTarget(0) = centroidTargetX;
        centroidTarget(1) = centroidTargetY;

        /**
        Subtract centroids
        */
        // Model
        for(int i = 0; i<modelDataSet.size(); i++)
        {
            modelDataSet[i].x -= centroidModelX;
            modelDataSet[i].y -= centroidModelY;
        }
        // Target
        for(int i = 0; i<targetDataSet.size(); i++)
        {
            targetDataSet[i].x -= centroidTargetX;
            targetDataSet[i].y -= centroidTargetY;
        }

        /**
        Calculate M
        */
        // Model
        MatrixXd MModel(2, modelDataSet.size());
        for(int i = 0; i<modelDataSet.size(); i++)
        {
            MModel(0, i) = modelDataSet[i].x;
            MModel(1, i) = modelDataSet[i].y;
        }
        // Target
        MatrixXd MTarget(2, targetDataSet.size());
        for(int i = 0; i<targetDataSet.size(); i++)
        {
            MTarget(0, i) = targetDataSet[i].x;
            MTarget(1, i) = targetDataSet[i].y;
        }

        /**
        Calculate H
        */
        MatrixXd H = MModel*MTarget.transpose();

        /**
        Calculate R
        */
        JacobiSVD<MatrixXd> svd(H,Eigen::ComputeThinU | Eigen::ComputeThinV);
        MatrixXd U = svd.matrixU();
        MatrixXd V = svd.matrixV();
        MatrixXd R = V*U.transpose();

        /**
        Calculate t
        */
        VectorXd t = centroidTarget-R*centroidModel;

        correction(0) = t(0);
        correction(1) = t(1);
        correction(2) = acos(R(0,0));
    }

    return correction;
}

