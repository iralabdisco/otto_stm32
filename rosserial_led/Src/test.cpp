#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#include "test.hpp"

void write(uint8_t* data, int length, UART_HandleTypeDef huart) {
	HAL_UART_Transmit(&huart, data, length, 100);
}
