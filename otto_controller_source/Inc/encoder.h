#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

class Encoder {
 public:
  TIM_HandleTypeDef* timer_;
  uint32_t elapsed_millis;
  uint32_t kTicksPerRevolution = 148000;
  float kPi = 3.14159;
  float kWheelCircumference = 0.7539; //in meters

  Encoder(){
    timer_ = NULL;
  }

  Encoder(TIM_HandleTypeDef* timer);

  void Setup();

  int GetCount() {
    return __HAL_TIM_GET_COUNTER(timer_);
  }

  void ResetCount() {
    __HAL_TIM_SET_COUNTER(timer_, 0);
  }

  float GetMeters();
  float GetLinearVelocity();

};
#endif
