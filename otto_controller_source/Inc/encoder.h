#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f7xx_hal_tim.h"

class Encoder {
 public:
  TIM_HandleTypeDef timer_;

  Encoder(TIM_HandleTypeDef timer);

  int GetCount();
  void ResetCount();
};
#endif
