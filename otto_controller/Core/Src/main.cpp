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

#include "control/encoder.h"
#include "control/odometry.h"
#include "control/motor_controller.h"
#include "control/pid.h"

#include "protobuf/otto_communication.pb.h"
#include "protobuf/pb_encode.h"
#include "protobuf/pb_decode.h"

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
int mode = 0;  //setup mode

uint8_t proto_buffer_rx[50];
pb_istream_t in_pb_stream;

VelocityCommand vel_cmd;
size_t velocity_cmd_length;
bool rx_status;

uint8_t proto_buffer_tx[100];
pb_ostream_t out_pb_stream;

StatusMessage status_msg;
size_t status_msg_length;
bool tx_status;
float previous_tx_millis;

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
  MX_GPIO_Init();
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

  //protobuffer messages init
  vel_cmd = VelocityCommand_init_zero;
  status_msg = StatusMessage_init_zero;

  //Enables TIM6 interrupt (used for periodic transmission of the odometry)
  HAL_TIM_Base_Start_IT(&htim6);

  //Enables UART RX interrupt
  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &proto_buffer_rx,
                       VelocityCommand_size);

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
  }

  //TIMER 10Hz Transmit
  if (htim->Instance == TIM6) {

    pb_ostream_t stream = pb_ostream_from_buffer(proto_buffer_tx, sizeof(proto_buffer_tx));

    float left_wheel = left_encoder.GetLinearVelocity();
    float right_wheel = right_encoder.GetLinearVelocity();

    odom.FromWheelVelToOdom(1, -1);

    status_msg.linear_velocity = odom.GetLinearVelocity();
    status_msg.angular_velocity = odom.GetAngularVelocity();

    float current_tx_millis = HAL_GetTick();
    status_msg.delta_millis = current_tx_millis - previous_tx_millis;
    previous_tx_millis = current_tx_millis;

    status_msg.status = 3;

    pb_encode(&stream, StatusMessage_fields, &status_msg);

    HAL_UART_Transmit_DMA(&huart6,(uint8_t*) &proto_buffer_tx, StatusMessage_size);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
//  size_t buffer_size = sizeof(proto_buffer_rx);
//  uint8_t buffer_copy[buffer_size];
//  memcpy((void *) &buffer_copy, &proto_buffer_rx, buffer_size);
//
//  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &proto_buffer_rx,
//                         velocity_cmd_length);
//  mode++;

  float linear_velocity;
  float angular_velocity;

  pb_istream_t stream = pb_istream_from_buffer(proto_buffer_rx,
                                               VelocityCommand_size);

  bool status = pb_decode(&stream, VelocityCommand_fields, &vel_cmd);

  if (status) {
    linear_velocity = vel_cmd.linear_velocity;
    angular_velocity = vel_cmd.angular_velocity;

    odom.FromCmdVelToSetpoint(linear_velocity, angular_velocity);

    float left_setpoint = odom.GetLeftVelocity();
    float right_setpoint = odom.GetRightVelocity();

//    left_pid.set(left_setpoint);
//    right_pid.set(right_setpoint);

    left_pid.set(0);
    right_pid.set(0);

    float cross_setpoint = left_setpoint - right_setpoint;
//    cross_pid.set(cross_setpoint);

    cross_pid.set(0);

  }

  HAL_UART_Receive_DMA(&huart6, (uint8_t*) &proto_buffer_rx,
                       VelocityCommand_size);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  //Blue user button on the NUCLEO board
  if (GPIO_Pin == GPIO_PIN_13) {
    if (mode == 0) {
      mode = 1;
      //Enables TIM3 interrupt (used for PID control)
      HAL_TIM_Base_Start_IT(&htim3);

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
