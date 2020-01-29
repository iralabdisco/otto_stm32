#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "constants.h"

class Encoder {
 public:
  TIM_HandleTypeDef *timer_;
  uint32_t previous_millis_;
  uint32_t current_millis_;
  int32_t ticks_;  //if negative the wheel is going backwards
  float wheel_circumference_;

  Encoder() {
    timer_ = NULL;
    wheel_circumference_ = 0;
  }

  Encoder(TIM_HandleTypeDef *timer, float wheel_circ);

  void Setup();

  int GetCount() {
    int count = ((int) __HAL_TIM_GET_COUNTER(this->timer_)
        - ((this->timer_->Init.Period) / 2));
    return count;
  }

  void ResetCount() {
    //set counter to half its maximum value
    __HAL_TIM_SET_COUNTER(timer_, (timer_->Init.Period / 2));
  }

  void UpdateValues();

  float GetMeters();

  float GetLinearVelocity();

};
#endif
