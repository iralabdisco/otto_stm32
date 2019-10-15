#include "encoder.h"

Encoder::Encoder(TIM_HandleTypeDef* timer) {
  timer_ = timer;
}

void Encoder::Setup() {
  HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
  this->ResetCount();
  this->previous_millis = 0;
  this->current_millis = HAL_GetTick();
}

void Encoder::UpdateValues() {
  this->previous_millis = this->current_millis;
  this->current_millis = HAL_GetTick();
  this->ticks = this->GetCount() - 2147483648;
  this->ResetCount();
}

float Encoder::GetMeters() {
  uint32_t ticks = this->GetCount();
  float meters = ((float) ticks * kWheelCircumference) / kTicksPerRevolution;
  this->ResetCount();
  return meters;
}

float Encoder::GetLinearVelocity() {
  float meters = this->GetMeters();
  float linear_velocity = meters
      / ((this->current_millis - previous_millis) / 1000);
  return linear_velocity;
}

