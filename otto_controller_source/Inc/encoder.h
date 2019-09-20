#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

class Encoder {
 public:
  TIM_HandleTypeDef* timer_;

  Encoder(TIM_HandleTypeDef* timer);

  int GetCount() {
    return __HAL_TIM_GET_COUNTER(timer_);
  }

  void ResetCount() {
    __HAL_TIM_SET_COUNTER(timer_, 0);
  }

};
#endif
