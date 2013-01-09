#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
class robotpospub
{

public:
	    robotpospub(){}
 double robotxx;
 double robotyy;
 double robotyyaw;
  void robotpos(double x, double y, double roll, double pitch, double yaw);
};


