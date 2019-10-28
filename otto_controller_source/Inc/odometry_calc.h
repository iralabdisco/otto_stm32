#ifndef ODOMETRY_CALC_H
#define ODOMETRY_CALC_H

#include "encoder.h"
#include "nav_msgs/Odometry.h"

class OdometryCalc{
  public:
    Encoder left_encoder_;
    Encoder right_encoder_;
    float theta;
    nav_msgs::Odometry odometry_;
    float kBaseline;

  OdometryCalc(){
    left_encoder_ = NULL;
    right_encoder_ = NULL;
    theta = 0;

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
    odometry_.twist.twist.angular; //TODO
    odometry_.twist.twist.linear; //TODO

    kBaseline = 0.35; //in meters
  }

  OdometryCalc(Encoder left, Encoder right){
    Encoder left_encoder_ = left;
    Encoder right_encoder_ = right;
  }

  void OdometryUpdateMessage();

};

#endif
