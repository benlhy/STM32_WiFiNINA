/*
 * mcp4922.c
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#include "mcp4922.h"

void init_mcp4922(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // set CS pin high
}
void set_A_mcp4922(uint16_t val) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // CS pin low, ready to send
	uint8_t buf[2] = {A_OPTIONS | val>>8, 0xFF & val };
	HAL_SPI_Transmit(&hspi1, buf, 2, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}

void set_B_mcp4922(uint16_t val) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // CS pin low, ready to send
	uint8_t buf[2] = {B_OPTIONS | val>>8, 0xFF & val };
	HAL_SPI_Transmit(&hspi1, buf, 2, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}

