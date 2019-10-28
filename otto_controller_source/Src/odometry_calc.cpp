#include "odometry_calc.h"
#include <cmath>

void OdometryCalc::OdometryUpdateMessage(){
  float left_velocity = left_encoder_.GetLinearVelocity();
  float right_velocity = right_encoder_.GetLinearVelocity();

  float x = odometry_.pose.pose.position.x;
  float y = odometry_.pose.pose.position.y;

  //verificato che delta_r == delta_l
  float delta_time = left_encoder_.current_millis_ -
      left_encoder_.previous_millis_;

  float linear_velocity = (left_velocity + right_velocity) / 2;
  float angular_velocity = (right_velocity - left_velocity) / kBaseline;

  float diff = angular_velocity / delta_time;

  float r = (kBaseline / 2) * ((right_velocity + left_velocity) /
      (right_velocity - left_velocity));

  float icc_x = x - r * std::sin(theta);
  float icc_y = y + r * std::cos(theta);

  float new_x = std::cos(diff) * (x - icc_x) -
      std::sin(diff) * (y - icc_y) + icc_x;

  float new_y = std::sin(diff) * (y - icc_y) +
      std::cos(diff) * (y - icc_y) + icc_y;

  float new_theta = theta + diff;

  return;
}
