#include "encoder.h"

Encoder::Encoder(TIM_HandleTypeDef* timer) {
  timer_ = timer;
  HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
}

