#include "mainpp.h"

ros::Subscriber<std_msgs::UInt8> led_sub("led", &led_cb);
ros::NodeHandle nh;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

void setup(void) {
	nh.initNode();
	nh.subscribe(led_sub);
}

void loop(void) {
	nh.spinOnce();
	HAL_Delay(1000);
}

void led_cb(const std_msgs::UInt8& msg) {
	int i = msg.data;
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_15);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_15);

}
