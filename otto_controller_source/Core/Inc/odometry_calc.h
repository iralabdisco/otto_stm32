#ifndef ODOMETRY_CALC_H
#define ODOMETRY_CALC_H

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>
#include <string>

#include "main.h"
#include "encoder.h"

class OdometryCalc {
public:
	Encoder left_encoder_;
	Encoder right_encoder_;
	float theta_;
	nav_msgs::Odometry odometry_;
	float kBaseline;

	OdometryCalc() {
		left_encoder_ = NULL;
		right_encoder_ = NULL;
		theta_ = 0;

		// odometry_.pose.covariance = 0.0;
		odometry_.pose.pose.position.x = 0.0;
		odometry_.pose.pose.position.y = 0.0;
		odometry_.pose.pose.position.z = 0.0;

		//orientation è il quaternione
		odometry_.pose.pose.orientation.x = 0.0;
		odometry_.pose.pose.orientation.y = 0.0;
		odometry_.pose.pose.orientation.z = 0.0;
		odometry_.pose.pose.orientation.w = 0.0;

		//odometry_.twist.covariance = 0.0;
		odometry_.twist.twist.angular.z = 0;
		odometry_.twist.twist.linear.x = 0;

		kBaseline = 0.35; //in meters
	}

	OdometryCalc(Encoder left, Encoder right){

		left_encoder_ = left;
		right_encoder_ = right;

		theta_ = 0;

		odometry_.header.frame_id = "odom";
		odometry_.child_frame_id = "base_link";

		// odometry_.pose.covariance = 0.0;
		odometry_.pose.pose.position.x = 0.0;
		odometry_.pose.pose.position.y = 0.0;
		odometry_.pose.pose.position.z = 0.0;

		//orientation è il quaternione
		odometry_.pose.pose.orientation.x = 0.0;
		odometry_.pose.pose.orientation.y = 0.0;
		odometry_.pose.pose.orientation.z = 0.0;
		odometry_.pose.pose.orientation.w = 0.0;

		//odometry_.twist.covariance = 0.0;
		odometry_.twist.twist.angular.z = 0;
		odometry_.twist.twist.linear.x = 0;

		kBaseline = 0.35; //in meters
	}

	void OdometryUpdateMessage();

};

#endif
