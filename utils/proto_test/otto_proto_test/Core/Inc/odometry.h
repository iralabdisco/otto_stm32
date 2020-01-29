#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "constants.h"

class Odometry {
 private:

  float left_velocity_;
  float right_velocity_;
  float linear_velocity_;
  float angular_velocity_;


 public:
  Odometry() {
    left_velocity_ = 0;
    right_velocity_ = 0;
    linear_velocity_ = 0;
    angular_velocity_ = 0;
  }

  void UpdateValuesFromVel(float linear_vel, float angular_vel) {
    left_velocity_ = linear_vel - (BASELINE * angular_vel)/2;
    right_velocity_ = linear_vel + (BASELINE * angular_vel)/2;
  }

  void UpdateValuesFromWheels(float left_velocity, float right_velocity){
    linear_velocity_ = (left_velocity + right_velocity)/2;
    angular_velocity_ = (right_velocity - left_velocity)/BASELINE;
  }


  float GetLeftVelocity(){
    return left_velocity_;
  }
  float GetRightVelocity(){
    return right_velocity_;
  }
  float GetLinearVelocity(){
    return linear_velocity_;
  }
  float GetAngularVelocity(){
    return angular_velocity_;
  }

};

#endif
