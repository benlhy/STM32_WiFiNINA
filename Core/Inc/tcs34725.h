/*
 * tcs34725.h
 *
 *  Created on: Jul 31, 2020
 *      Author: ben
 */

#ifndef INC_TCS34725_H_
#define INC_TCS34725_H_

#include "stm32f7xx_hal.h"
#include "main.h"

#define TCS34725_I2C_ADDR 0x29
#define TCS34725_ENABLE (0x00)
#define TCS34725_ENABLE_PON (0x01)
#define TCS34725_ENABLE_AEN (0x02)
#define TCS34725_COMMAND_BIT (0x80)
#define TCS34725_ATIME (0x01)
#define TCS34725_INTEGRATIONTIME_2_4MS (0xFF)
#define TCS34725_CONTROL (0x0F)

#define TCS34725_GAIN_1X (0x00)
#define TCS34725_COLOR_START (0x14)

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef ret;


void init_tcs34725(void);

void get_crgb(uint16_t* c,uint16_t* r,uint16_t* g,uint16_t* b);


#endif /* INC_TCS34725_H_ */
