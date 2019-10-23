#include "odometry_calc.h"
#include <cmath>

void OdometryCalc::OdometryUpdateMessage(){
  float left_velocity = left_encoder_.GetLinearVelocity();
  float right_velocity = right_encoder_.GetLinearVelocity();
  float delta_time = 0.0; //TODO quale delta prendo?

  float linear_velocity = (left_velocity + right_velocity) / 2;
  float angular_velocity = (right_velocity - left_velocity) / kBaseline;

  float r = (kBaseline / 2) * ((right_velocity + left_velocity) /
      (right_velocity - left_velocity) );

  float icc_x;
  float icc_y;

  float new_x;
  float new_y;
  float new_theta;

  return;
}
