#include "encoder.h"

Encoder::Encoder(TIM_HandleTypeDef *timer) {
  timer_ = timer;
}

void Encoder::Setup() {
  HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
  this->ResetCount();
  this->previous_millis_ = 0;
  this->current_millis_ = HAL_GetTick();
}

void Encoder::UpdateValues() {
  this->previous_millis_ = this->current_millis_;
  this->current_millis_ = HAL_GetTick();
  this->ticks_ = this->GetCount();
  this->ResetCount();
}

float Encoder::GetMeters() {
  this->UpdateValues();
  float meters = ((float) this->ticks_ * WHEEL_CIRCUMFERENCE)
      / TICKS_PER_REVOLUTION;
  return meters;
}

float Encoder::GetLinearVelocity() {
  this->UpdateValues();
  float meters = ((float) this->ticks_ * WHEEL_CIRCUMFERENCE)
      / TICKS_PER_REVOLUTION;
  float deltaTime = this->current_millis_ - this->previous_millis_;
  if (deltaTime == 0)
    return 0;
  float linear_velocity = (meters / (deltaTime / 1000));
  return linear_velocity;
}

