#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

class Encoder {
 public:
  TIM_HandleTypeDef* timer_;
  uint32_t previous_millis_;
  uint32_t current_millis_;
  int32_t ticks_; //if negative the wheel is going backwards

  uint32_t kTicksPerRevolution = 74000; //x2 resolution
  float kPi = 3.14159;
  float kWheelCircumference = 0.7539; //in meters

  Encoder(){
    timer_ = NULL;
  }

  Encoder(TIM_HandleTypeDef* timer);

  void Setup();

  int GetCount() {
    int count = ((int)__HAL_TIM_GET_COUNTER(timer_) - ((timer_->Init.Period)/2));
    return count;
  }

  void ResetCount() {
    //set counter to half its maximum value
    __HAL_TIM_SET_COUNTER(timer_, (timer_->Init.Period)/2);
  }

  void UpdateValues();

  float GetMeters();

  float GetLinearVelocity();

};
#endif
