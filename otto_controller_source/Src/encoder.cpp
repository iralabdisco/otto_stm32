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
  this->ticks = this->GetCount();
  this->ResetCount();
}

float Encoder::GetMeters(){
  this->UpdateValues();
  float meters = ((float) this->ticks * kWheelCircumference)
      / kTicksPerRevolution;
  return meters;
}

float Encoder::GetLinearVelocity() {
  this->UpdateValues();
  float meters = ((float) this->ticks * kWheelCircumference)
      / kTicksPerRevolution;
  float deltaTime = this->current_millis - this->previous_millis;
  float linear_velocity = (meters / (deltaTime / 1000));
  return linear_velocity;
}

