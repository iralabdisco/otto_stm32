#ifndef ODOMETRY_H
#define ODOMETRY_H

class Odometry {
 private:

  float left_velocity_;
  float right_velocity_;
  float baseline_;

 public:
  Odometry() {
    left_velocity_ = 0;
    right_velocity_ = 0;
    baseline_ = 0;
  }

  Odometry(float baseline) {
    left_velocity_ = 0;
    right_velocity_ = 0;
    baseline_ = baseline;
  }

  void UpdateValues(float linear_vel, float angular_vel) {
    left_velocity_ = linear_vel - (baseline_ * angular_vel) / 2;
    right_velocity_ = linear_vel + (baseline_ * angular_vel) / 2;
  }

  float GetLeftVelocity() {
    return left_velocity_;
  }
  float GetRightVelocity() {
    return right_velocity_;
  }

};

#endif
