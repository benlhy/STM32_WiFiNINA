/*
 * wifinina.cpp
 *
 *  Created on: Aug 4, 2020
 *      Author: ben
 */

#include "wifinina.h"

int16_t WIFININA_SLAVEGPIO0 = GPIO_PIN_0;
int16_t WIFININA_SLAVEREADY = GPIO_PIN_13;
int16_t WIFININA_SLAVESELECT =  GPIO_PIN_14;
int16_t WIFININA_SLAVERESET = GPIO_PIN_15;


void pinMode(GPIO_TypeDef* port, uint32_t pin, uint32_t mode, uint32_t pull) {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = pull;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void delay(uint32_t val) {
#ifdef CMSIS_OS_H_
	osDelay(val);
#else
	HAL_Delay(val);

#endif
}

GPIO_PinState digitalRead(GPIO_TypeDef* port,  uint32_t pin) {
	return HAL_GPIO_ReadPin(port, pin);
}
void digitalWrite(GPIO_TypeDef* port,  uint32_t pin, GPIO_PinState state) {
	HAL_GPIO_WritePin(port, pin, state);
}
char SPITransfer(volatile char data) {
	  uint8_t txBuffer = (uint8_t)data;
	  uint8_t rxBuffer;
	  HAL_SPI_TransmitReceive(&hspi1, &txBuffer, &rxBuffer, 1, 10);
	  return rxBuffer;

}

