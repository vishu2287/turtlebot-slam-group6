#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <robotpos.hpp>

double robotxx = 0;
double robotyy = 0;
double robotyyaw = 0;

void robotpospub::robotpos(double x, double y, double roll, double pitch, double yaw) {
	std::cout << "Robotpublisher created" << std::endl;
	robotxx = x;
	robotyy = y;
	robotyyaw = yaw;
    int32_t publish_rate_ = 75;
	tf::TransformBroadcaster tf_br_;
	tf::StampedTransform tf_map_to_baselink_;
	tf::TransformBroadcaster tf_br_1;
	tf::StampedTransform tf_map_to_odom_;
// set up parent and child frames
	tf_map_to_baselink_.frame_id_ = std::string("world");
	tf_map_to_baselink_.child_frame_id_ = std::string("base_link");
	tf_map_to_odom_.frame_id_ = std::string("world");
	tf_map_to_odom_.child_frame_id_ = std::string("odom");

// set up publish rate
	ros::Rate loop_rate(publish_rate_);
//grid_x = (unsigned int)((map_x - map.info.origin.position.x) / map.info.resolution)
//grid_y = (unsigned int)((map_y - map.info.origin.position.y) / map.info.resolution)
// main loop

	while (ros::ok()) {
		// time stamp
		tf_map_to_baselink_.stamp_ = ros::Time::now();
		tf_map_to_odom_.stamp_ = ros::Time::now();

		// specify actual transformation vectors from odometry
		// NOTE: zeros have to be substituted with actual variable data
		tf_map_to_baselink_.setOrigin(tf::Vector3(robotxx, robotyy, 0.0f));
		tf_map_to_odom_.setOrigin(tf::Vector3(robotxx, robotyy, 0.0f));
		if (yaw < 0) {
			yaw = 2 * M_PI + yaw;
		}
		tf_map_to_baselink_.setRotation(tf::createQuaternionFromYaw(robotyyaw));
		tf_map_to_odom_.setRotation(tf::createQuaternionFromYaw(robotyyaw));

		// broadcast transform
		tf_br_.sendTransform(tf_map_to_baselink_);
		tf_br_1.sendTransform(tf_map_to_odom_);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
