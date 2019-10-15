#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

class Encoder {
 public:
  TIM_HandleTypeDef* timer_;
  uint32_t previous_millis;
  uint32_t current_millis;
  int32_t ticks; //if negative the wheel is going backwards

  uint32_t kTicksPerRevolution = 148000;
  float kPi = 3.14159;
  float kWheelCircumference = 0.7539; //in meters

  Encoder(){
    timer_ = NULL;
  }

  Encoder(TIM_HandleTypeDef* timer);

  void Setup();

  int GetCount() {
    return (__HAL_TIM_GET_COUNTER(timer_) - 2147483648);
  }

  void ResetCount() {
    //set counter to half its maximum value
    __HAL_TIM_SET_COUNTER(timer_, 2147483648);
  }

  void UpdateValues();

  float GetMeters();

  float GetLinearVelocity();

};
#endif
