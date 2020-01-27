#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "constants.h"

class Odometry {
 private:

  float left_velocity_;
  float right_velocity_;


 public:
  Odometry() {
    left_velocity_ = 0;
    right_velocity_ = 0;
  }

  void UpdateValues(float linear_vel, float angular_vel) {
    left_velocity_ = linear_vel - (BASELINE * angular_vel)/2;
    right_velocity_ = linear_vel + (BASELINE * angular_vel)/2;
  }

  float GetLeftVelocity(){
    return left_velocity_;
  }
  float GetRightVelocity(){
    return right_velocity_;
  }

};

#endif
