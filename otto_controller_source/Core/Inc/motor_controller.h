#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "main.h"

class MotorController {
 public:
  GPIO_TypeDef *sleep_gpio_port_;
  uint16_t sleep_pin_;
  GPIO_TypeDef *dir_gpio_port_;
  uint16_t dir_pin_;
  TIM_HandleTypeDef *pwm_timer_;
  uint32_t pwm_channel_;

  MotorController(GPIO_TypeDef *sleep_gpio_port, uint16_t sleep_pin,
                  GPIO_TypeDef *dir_gpio_port, uint16_t dir_pin,
                  TIM_HandleTypeDef *pwm_timer, uint32_t pwm_channel) {
    this->sleep_gpio_port_ = sleep_gpio_port;
    this->sleep_pin_ = sleep_pin;
    this->dir_gpio_port_ = dir_gpio_port;
    this->dir_pin_ = dir_pin;
    this->pwm_timer_ = pwm_timer;
    this->pwm_channel_ = pwm_channel;
  }

  void setup() {
    HAL_TIM_PWM_Start(pwm_timer_, pwm_channel_);
  }

  void set_speed(int duty_cycle) {
    if (duty_cycle >= 0) {
      HAL_GPIO_WritePin(dir_gpio_port_, dir_pin_, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_channel_, duty_cycle);
    } else {
      HAL_GPIO_WritePin(dir_gpio_port_, dir_pin_, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_channel_, -duty_cycle);
    }

  }

  void brake() {
    __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_channel_, 0);
  }

  void coast() {
    HAL_GPIO_WritePin(sleep_gpio_port_, sleep_pin_, GPIO_PIN_RESET);
  }
};
#endif
