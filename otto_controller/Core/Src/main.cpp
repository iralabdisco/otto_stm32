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
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "control/encoder.h"
#include "control/odometry.h"
#include "control/motor_controller.h"
#include "control/pid.h"

#include "communication/otto_messages.h"

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
float baseline = 0.435;
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

float left_setpoint = 0;
float right_setpoint = 0;

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
VelocityMessage vel_msg;
StatusMessage status_msg;

bool tx_status;
float previous_tx_millis;

int otto_status = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t test_receive[] = { 0, 0 };
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  MX_CRC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  left_encoder.Setup();
  right_encoder.Setup();

  left_motor.Setup();
  right_motor.Setup();

  //right and left motors have the same parameters
  uint32_t max_dutycycle = *(&htim4.Instance->ARR);
  pid_min = -(int) max_dutycycle;
  pid_max = (int) max_dutycycle;

  left_pid.Config(180, 200, 0, pid_min, pid_max);
  right_pid.Config(185, 195, 0, pid_min, pid_max);
  cross_pid.Config(50, 20, 0, pid_min, pid_max);

  left_motor.Coast();
  right_motor.Coast();

  //Enables UART RX interrupt
  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &vel_msg, 12);

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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  //TIMER 100Hz PID control
  left_velocity = left_encoder.GetLinearVelocity();
  left_dutycycle = left_pid.Update(left_velocity);
  left_motor.SetSpeed(left_dutycycle);

  right_velocity = right_encoder.GetLinearVelocity();
  right_dutycycle = right_pid.Update(right_velocity);
  right_motor.SetSpeed(right_dutycycle);

  float difference = left_velocity - right_velocity;

  int cross_dutycycle = cross_pid.Update(difference);

  left_dutycycle += cross_dutycycle;
  right_dutycycle -= cross_dutycycle;

}

uint8_t tx_flag = 1;
HAL_StatusTypeDef stato;
uint32_t crc_rx = 0;
uint32_t crc_tx = 0;
uint8_t crc_rx_ok = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
  /*
   * Manage received message
   */

  float linear_velocity;
  float angular_velocity;

  linear_velocity = vel_msg.linear_velocity;
  angular_velocity = vel_msg.angular_velocity;

  crc_rx = HAL_CRC_Calculate(&hcrc, (uint32_t*) &vel_msg, 8);

  if (crc_rx == vel_msg.crc) {
    crc_rx_ok = 1;
  }

  odom.FromCmdVelToSetpoint(linear_velocity, angular_velocity);

  left_setpoint = odom.GetLeftVelocity();
  right_setpoint = odom.GetRightVelocity();

  left_pid.Set(left_setpoint);
  right_pid.Set(right_setpoint);

  float cross_setpoint = left_setpoint - right_setpoint;
  cross_pid.Set(cross_setpoint);

  if (otto_status == 0) {
    //Enables TIM6 interrupt (used for PID control)
    HAL_TIM_Base_Start_IT(&htim6);

    //running
    otto_status = 3;
  }

  /*
   * Manage new transmission
   */

  float left_wheel = left_encoder.GetLinearVelocity();
  float right_wheel = right_encoder.GetLinearVelocity();

  odom.FromWheelVelToOdom(left_wheel, right_wheel);

  // used to debug
  //odom.FromWheelVelToOdom(left_setpoint, right_setpoint);

  status_msg.left_ticks = 22;
  status_msg.right_ticks = 27;

  float current_tx_millis = HAL_GetTick();
  status_msg.delta_millis = current_tx_millis - previous_tx_millis;
  previous_tx_millis = current_tx_millis;

  status_msg.status = 3;

  crc_tx = HAL_CRC_Calculate(&hcrc, (uint32_t*) &status_msg, 12);

  status_msg.crc = crc_tx;

  if (tx_flag == 1) {
    stato = HAL_UART_Transmit_DMA(&huart6, (uint8_t*) &status_msg,
                                  sizeof(status_msg));
    tx_flag = 0;
  }

  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &vel_msg, sizeof(vel_msg));
}

uint8_t tx_ok = 0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
  tx_ok += 1;
  tx_flag = 1;
}

uint8_t tx_err = 0;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
  tx_err += 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  //Blue user button on the NUCLEO board
  if (GPIO_Pin == user_button_Pin) {
    //TODO ci può servire il bottone blu?
  } else if (GPIO_Pin == fault1_Pin) {
    left_motor.Brake();
    right_motor.Brake();
    //stop TIM6 interrupt (used for PID control)
    HAL_TIM_Base_Stop_IT(&htim6);

    //loops forever, manual reset is needed
    while (1)
      ;

  } else if (GPIO_Pin == fault2_Pin) {

    left_motor.Brake();
    right_motor.Brake();
    //stop TIM6 interrupt (used for PID control)
    HAL_TIM_Base_Stop_IT(&htim6);

    //loops forever, manual reset is needed
    while (1)
      ;
  }

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
