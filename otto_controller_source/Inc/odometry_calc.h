#ifndef ODOMETRY_CALC_H
#define ODOMETRY_CALC_H

#include "encoder.h"
#include "nav_msgs/Odometry.h"

class OdometryCalc{
  public:
    Encoder left_encoder_;
    Encoder right_encoder_;
    nav_msgs::Odometry odometry_;

  OdometryCalc(){
    left_encoder_ = NULL;
    right_encoder_ = NULL;
  }

  OdometryCalc(Encoder left, Encoder right){
    Encoder left_encoder_ = left;
    Encoder right_encoder_ = right;
  }



};


#endif
