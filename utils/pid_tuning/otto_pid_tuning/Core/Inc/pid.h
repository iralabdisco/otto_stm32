#ifndef PID_H
#define PID_H

#define WINDOW_SIZE 500

class Pid {
 public:
  //PID constants
  float kp_;
  float ki_;
  float kd_;

  float error_;
  float setpoint_;

  //needed for integral term
  float error_sum_array_[WINDOW_SIZE];
  int error_sum_index_;

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
    this->error_sum_index_ = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
      this->error_sum_array_[i] = 0;
    }

    this->min_ = -790;
    this->max_ = 790;

  }

  void config(float kp, float ki, float kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;

    this->error_ = 0;
    this->setpoint_ = 0;

    this->previous_error_ = 0;
    this->error_sum_index_ = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
      this->error_sum_array_[i] = 0;
    }
  }

  void set(float setpoint) {
    this->setpoint_ = setpoint;
  }

  int update(float measure) {

    this->error_ = this->setpoint_ - measure;

    //proportional term
    float output = this->error_ * this->kp_;

    //integral term with antiwindup
//    if (this->error_sum_index_ == WINDOW_SIZE) {
//      this->error_sum_array_[0] = this->error_;
//      this->error_sum_index_ = 1;
//    } else {
//      this->error_sum_array_[this->error_sum_index_] = this->error_;
//      this->error_sum_index_++;
//    }
//
//    float error_sum = 0;
//    for (int i = 0; i < WINDOW_SIZE; i++) {
//      error_sum += this->error_sum_array_[i];
//    }
//
//    output += error_sum * this->ki_;

    //integral term without windup

    error_sum_array_[0] += this->error_;
    output += error_sum_array_[0] * this->ki_;



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
