#ifndef PID_H
#define PID_H

#include "constants.h"

class Pid {
 public:
  //PID constants
  float kp_;
  float ki_;
  float kd_;

  float error_;
  float setpoint_;

  //needed for integrative term
  int error_sum_;

  //needed for derivative term
  float previous_error_;

  int min_;
  int max_;

  Pid(float kp, float ki, float kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;

    this->error_ = 0;
    this->setpoint_ = 0;

    this->previous_error_ = 0;
    this->error_sum_ = 0;

    this->min_ = -MAX_DUTY_CYCLE;
    this->max_ = MAX_DUTY_CYCLE;

  }

  void config(float kp, float ki, float kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;

    this->error_ = 0;
    this->setpoint_ = 0;

    this->previous_error_ = 0;
    this->error_sum_ = 0;

  }

  void set(float setpoint) {
    this->setpoint_ = setpoint;
  }

  int update(float measure) {

    this->error_ = this->setpoint_ - measure;

    //proportional term
    float output = this->error_ * this->kp_;

    //integral term without windup
    error_sum_ += this->error_;
    output += error_sum_ * this->ki_;

    //derivative term
    output += (this->error_ - this->previous_error_) * kd_;
    this->previous_error_ = this->error_;

    int integer_output = static_cast<int> (output);

    if(integer_output > this->max_)
      integer_output = this->max_;
    else if (integer_output < this->min_)
      integer_output = this->min_;

    return integer_output;

  }
};
#endif
