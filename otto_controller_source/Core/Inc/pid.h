#ifndef PID_H
#define PID_H

class Pid {
 public:
  //PID constants
  float kp_;
  float ki_;
  float kd_;

  float error_;
  float setpoint_;

  //needed for integral term
  float error_sum_array_[10];
  int error_sum_index_;

  //needed for derivative term
  float previous_error_;

//  int min_;
//  int max_;

  Pid(float kp, float ki, float kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;

    this->error_ = 0;
    this->setpoint_ = 0;

    this->previous_error_ = 0;
    this->error_sum_index_ = 0;

    for (int i = 0; i < 10; i++) {
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

    //integral term

    if (this->error_sum_index_ == 10) {
      this->error_sum_array_[0] = this->error_;
      this->error_sum_index_ = 0;
    } else {
      this->error_sum_array_[this->error_sum_index_] = this->error_;
      this->error_sum_index_++;
    }

    float error_sum = 0;
    for (int i = 0; i < 10; i++) {
      error_sum += this->error_sum_array_[i];
    }


    output += error_sum * this->ki_;

    //TODO derivative term

    int integer_output = (int) output;

    return integer_output;

  }
};
#endif
