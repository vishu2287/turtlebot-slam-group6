#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>

void transformPoint(const tf::TransformListener& listener){
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "map"; //The top-most frame
  //transform object storing our robot's position
  tf::StampedTransform transform;
  try{
    geometry_msgs::PointStamped base_point;
    //listener.transformPoint("odom", laser_point, base_point);

	listener.lookupTransform("/map", "/odom", ros::Time(0), transform);
	// X and Y translation coordinate from the origin, where the robot started
 	double x = transform.getOrigin().x();
 	double y = transform.getOrigin().y();
	//Print out current translated position of the robot
	double turn = tf::getYaw(transform.getRotation());
	// = transform.getRotation();
	ROS_INFO("X Origin : %f Y Origin : %f current turnangle : %f",x,y,turn);

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
/*#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/Spawn.h>
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;

  tf::TransformListener listener;
  ros::Rate rate(10.0);
    tf::StampedTransform transform;
  while (ros::ok())
  {
geometry_msgs::PoseStamped pBase, pMap;
pBase.header.frame_id = "map";
pBase.pose.position.x = 0.0;
pBase.pose.position.y = 0.0;
pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
ros::Time current_transform = ros::Time::now();
listener.getLatestCommonTime(pBase.header.frame_id, "/odom", current_transform, NULL);
pBase.header.stamp = current_transform;
listener.transformPose("/odom", pBase, pMap);

    std_msgs::String msg;

    std::stringstream ss;
    ss << pMap << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
   ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
*/

/*#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

 /* ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = 
       node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel = 
       node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10); 

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/odom",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    turtlesim::Velocity vel_msg;
    vel_msg.angular = 4 * atan2(transform.getOrigin().y(),
                                transform.getOrigin().x());
    vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                 pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};*/
