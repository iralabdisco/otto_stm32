#include "odometry_calc.h"

void OdometryCalc::OdometryUpdateMessage(){
  float left_velocity = left_encoder_.GetLinearVelocity();
  float right_velocity = right_encoder_.GetLinearVelocity();

  float x = odometry_.pose.pose.position.x;
  float y = odometry_.pose.pose.position.y;

  //verificato che delta_r == delta_l
  float delta_time = left_encoder_.current_millis_ -
      left_encoder_.previous_millis_;

  // calcoli vari
  float linear_velocity = (left_velocity + right_velocity) / 2;
  float angular_velocity;
  if (right_velocity - left_velocity == 0)
    angular_velocity = 0;
  else
    angular_velocity = (right_velocity - left_velocity) / kBaseline;
  float diff = angular_velocity / delta_time;
  float r = (kBaseline / 2) * ((right_velocity + left_velocity) /
      (right_velocity - left_velocity));
  float icc_x = x - r * std::sin(theta_);
  float icc_y = y + r * std::cos(theta_);
  float new_x = std::cos(diff) * (x - icc_x) -
      std::sin(diff) * (y - icc_y) + icc_x;
  float new_y = std::sin(diff) * (y - icc_y) +
      std::cos(diff) * (y - icc_y) + icc_y;
  theta_ = theta_ + diff;
  geometry_msgs::Quaternion q = tf::createQuaternionFromYaw(theta_);

  //update msg
  odometry_.pose.pose.position.x = new_x;
  odometry_.pose.pose.position.y = new_y;
  odometry_.pose.pose.orientation.x = q.x;
  odometry_.pose.pose.orientation.y = q.y;
  odometry_.pose.pose.orientation.z = q.z;
  odometry_.pose.pose.orientation.w = q.w;
  odometry_.twist.twist.linear.x = linear_velocity;
  odometry_.twist.twist.angular.z = angular_velocity;

  return;
}
