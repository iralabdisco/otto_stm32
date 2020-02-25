#ifndef ODOMETRY_H
#define ODOMETRY_H

class Odometry {
 private:

  float left_setpoint_;
  float right_setpoint_;
  float linear_velocity_;
  float angular_velocity_;
  float baseline_;

 public:
  Odometry() {
    left_setpoint_ = 0;
    right_setpoint_ = 0;
    baseline_ = 0;
  }

  Odometry(float baseline) {
    left_setpoint_ = 0;
    right_setpoint_ = 0;
    baseline_ = baseline;
  }

  void FromCmdVelToSetpoint(float linear_vel, float angular_vel) {
    left_setpoint_ = linear_vel - (baseline_ * angular_vel) / 2;
    right_setpoint_ = linear_vel + (baseline_ * angular_vel) / 2;
  }

  void FromWheelVelToOdom(float left_wheel_vel, float right_wheel_vel){
    linear_velocity_ = (left_wheel_vel + right_wheel_vel)/2;
    angular_velocity_ = (right_wheel_vel - left_wheel_vel)/baseline_;
  }

  float GetLeftVelocity() {
    return left_setpoint_;
  }
  float GetRightVelocity() {
    return right_setpoint_;
  }
  float GetLinearVelocity(){
    return linear_velocity_;
  }
  float GetAngularVelocity(){
    return angular_velocity_;
  }

};

#endif
