/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "encoder.h"
#include "odometry.h"
#include "motor_controller.h"
#include "pid.h"
#include "communication_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Parameters
float baseline = 0.3;
int ticks_per_revolution = 148000;  //x4 resolution
float right_wheel_circumference = 0.783;  //in meters
float left_wheel_circumference = 0.789;  //in meters

//Odometry
Encoder right_encoder = Encoder(&htim5, right_wheel_circumference,
                                ticks_per_revolution);
Encoder left_encoder = Encoder(&htim2, left_wheel_circumference,
                               ticks_per_revolution);
Odometry odom = Odometry(baseline);

float left_velocity;
float right_velocity;

//PID
int pid_min = 0;
int pid_max = 0;

Pid left_pid(180, 200, 0, pid_min, pid_max);
Pid right_pid(185, 195, 0, pid_min, pid_max);
Pid cross_pid(50, 20, 0, pid_min, pid_max);

int left_dutycycle;
int right_dutycycle;

//MotorController
MotorController right_motor(sleep1_GPIO_Port,
sleep1_Pin,
                            dir1_GPIO_Port,
                            dir1_Pin,
                            &htim4, TIM_CHANNEL_4);
MotorController left_motor(sleep2_GPIO_Port,
sleep2_Pin,
                           dir2_GPIO_Port,
                           dir2_Pin,
                           &htim4, TIM_CHANNEL_3);

//Communication
uint8_t *tx_buffer;
uint8_t *rx_buffer;

velocity_msg vel_msg;
wheel_msg wheels_msg;

uint8_t mode = 0;  //setup mode

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  left_encoder.Setup();
  right_encoder.Setup();

  left_motor.setup();
  right_motor.setup();

  //right and left motors have the same parameters
  pid_min = -left_motor.max_dutycycle_;
  pid_max = left_motor.max_dutycycle_;

  left_pid.config(180, 200, 0, pid_min, pid_max);
  right_pid.config(185, 195, 0, pid_min, pid_max);
  cross_pid.config(50, 20, 0, pid_min, pid_max);

  left_motor.coast();
  right_motor.coast();

  tx_buffer = (uint8_t*) &wheels_msg;
  rx_buffer = (uint8_t*) &vel_msg;

  //Enables UART RX interrupt
  HAL_UART_Receive_DMA(&huart6, rx_buffer, 8);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  //TIMER 100Hz PID control
  if (htim->Instance == TIM3) {

    left_velocity = left_encoder.GetLinearVelocity();
    left_dutycycle = left_pid.update(left_velocity);
    left_motor.set_speed(left_dutycycle);

    right_velocity = right_encoder.GetLinearVelocity();
    right_dutycycle = right_pid.update(right_velocity);
    right_motor.set_speed(right_dutycycle);

    float difference = left_velocity - right_velocity;

    int cross_dutycycle = cross_pid.update(difference);

    left_dutycycle += cross_dutycycle;
    right_dutycycle -= cross_dutycycle;

    wheels_msg.left_vel = left_velocity;
    wheels_msg.right_vel = right_velocity;

  }

  //TIMER 2Hz Transmit
  if (htim->Instance == TIM6) {

    //TODO odometry
    HAL_UART_Transmit_IT(&huart6, tx_buffer, 8);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
  odom.UpdateValues(vel_msg.linear_velocity, vel_msg.angular_velocity);

  float left_setpoint = odom.GetLeftVelocity();
  float right_setpoint = odom.GetRightVelocity();

  left_pid.set(left_setpoint);
  right_pid.set(right_setpoint);

  float cross_setpoint = left_setpoint - right_setpoint;
  cross_pid.set(cross_setpoint);

  HAL_UART_Receive_IT(&huart6, rx_buffer, 8);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  //Blue user button on the NUCLEO board
  if (GPIO_Pin == GPIO_PIN_13) {
    mode++;
    if (mode == 0) {
      mode = 1;
      //Enables TIM3 interrupt (used for PID control)
      HAL_TIM_Base_Start_IT(&htim3);
      //Enables TIM6 interrupt (used for periodic transmission)
      HAL_TIM_Base_Start_IT(&htim6);

    }

  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameduty_cycleters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
