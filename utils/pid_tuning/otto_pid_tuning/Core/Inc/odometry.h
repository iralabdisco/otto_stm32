#ifndef ODOMETRY_CALC_H
#define ODOMETRY_CALC_H

#include "main.h"
#include "encoder.h"

class Odometry {
 public:
  Encoder left_encoder_;
  Encoder right_encoder_;
  float kBaseline;

  float linear_velocity;
  float angular_velocity;
  int delta_time;

  Odometry() {
    left_encoder_ = NULL;
    right_encoder_ = NULL;
    kBaseline = 0.35;  //in meters
  }

  Odometry(Encoder left, Encoder right) {

    left_encoder_ = left;
    right_encoder_ = right;
    kBaseline = 0.35;  //in meters
  }

  void UpdateValues() {
    float left_velocity = left_encoder_.GetLinearVelocity();
    float right_velocity = right_encoder_.GetLinearVelocity();

    //verificato che delta_r == delta_l
    this->delta_time = left_encoder_.current_millis_
        - left_encoder_.previous_millis_;

    this->linear_velocity = (left_velocity + right_velocity) / 2;
    this->angular_velocity = (right_velocity - left_velocity) / kBaseline;
    return;
  }

};

#endif
