/*
 * mcp4922.h
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#ifndef INC_MCP4922_H_
#define INC_MCP4922_H_

#include "stm32f7xx_hal.h"
#include "main.h"

#define A_OPTIONS 0x30
#define B_OPTIONS 0xB0

extern SPI_HandleTypeDef hspi1;

void init_mcp4922(void);
void set_A_mcp4922(uint16_t val);
void set_B_mcp4922(uint16_t val);

#endif /* INC_MCP4922_H_ */
