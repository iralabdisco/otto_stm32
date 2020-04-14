#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

class Encoder {
 private:
  TIM_HandleTypeDef *timer_;
  uint32_t previous_millis_;
  uint32_t current_millis_;
  int32_t ticks_;  //if negative the wheel is going backwards
  float wheel_circumference_;
  int ticks_per_revolution_;

 public:
  Encoder() {
    timer_ = NULL;
    wheel_circumference_ = 0;
    ticks_per_revolution_ = 0;
  }

  Encoder(TIM_HandleTypeDef *timer, float wheel_circ,
          int ticks_per_revolution) {
    timer_ = timer;
    wheel_circumference_ = wheel_circ;
    ticks_per_revolution_ = ticks_per_revolution;

  }

  int GetCount() {
    int count = ((int) __HAL_TIM_GET_COUNTER(this->timer_)
        - ((this->timer_->Init.Period) / 2));
    return count;
  }

  void ResetCount() {
    //set counter to half its maximum value
    __HAL_TIM_SET_COUNTER(timer_, (timer_->Init.Period / 2));
  }

  void Setup() {
    HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
    this->ResetCount();
    this->previous_millis_ = 0;
    this->current_millis_ = HAL_GetTick();
  }

  void UpdateValues() {
    this->previous_millis_ = this->current_millis_;
    this->current_millis_ = HAL_GetTick();
    this->ticks_ = this->GetCount();
    this->ResetCount();
  }

  float GetMeters() {
    float meters = ((float) this->ticks_ * this->wheel_circumference_)
        / ticks_per_revolution_;
    return meters;
  }

  float GetLinearVelocity() {
    this->UpdateValues();
    float meters = this->GetMeters();
    float deltaTime = this->current_millis_ - this->previous_millis_;
    if (deltaTime == 0)
      return 0;
    float linear_velocity = (meters / (deltaTime / 1000));
    return linear_velocity;
  }

};
#endif
