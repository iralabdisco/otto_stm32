#include "encoder.h"

Encoder::Encoder(TIM_HandleTypeDef timer){
  timer_ = timer;
}

Encoder::GetCount(){
  return __HAL_TIM_GetCounter(timer_);
}

Encoder::ResetCount(){
  __HAL_TIM_SetCounter(timer_, 0);
}
