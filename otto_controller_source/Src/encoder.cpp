#include "encoder.h"

Encoder::Encoder(TIM_HandleTypeDef* timer) {
  timer_ = timer;
  HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
  elapsed_millis = HAL_GetTick();
}

float Encoder::GetLinearVelocity(){
  uint32_t ticks = this->GetCount();
  uint32_t previous_millis = this->elapsed_millis;
  this->elapsed_millis = HAL_GetTick();
  float meters =  (ticks * kWheelCircumference) / kTicksPerRevolution;
  float linear_velocity = meters /
      ((this->elapsed_millis - previous_millis) / 1000);
  return linear_velocity;
}

