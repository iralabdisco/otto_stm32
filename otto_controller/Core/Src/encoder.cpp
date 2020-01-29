#include "encoder.h"

Encoder::Encoder(TIM_HandleTypeDef *timer, float wheel_circ) {
  timer_ = timer;
  wheel_circumference_ = wheel_circ;

}

//Encoder::Encoder(TIM_HandleTypeDef *timer, int ticks_per_meter) {
//  timer_ = timer;
//  ticks_per_meter_ = ticks_per_meter;
//
//}

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
  float meters = ((float) this->ticks_ * this->wheel_circumference_)
      / TICKS_PER_REVOLUTION;
  return meters;
}

//float Encoder::GetMeters() {
//  float meters = ((float) (this->ticks_) / this->ticks_per_meter_);
//  return meters;
//}


float Encoder::GetLinearVelocity() {
  this->UpdateValues();
  float meters = this->GetMeters();
  float deltaTime = this->current_millis_ - this->previous_millis_;
  if (deltaTime == 0)
    return 0;
  float linear_velocity = (meters / (deltaTime / 1000));
  return linear_velocity;
}

