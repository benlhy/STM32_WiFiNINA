/*
 * tcs34725.cpp
 *
 *  Created on: Jul 31, 2020
 *      Author: ben
 */


#include "tcs34725.h"

static uint8_t read8(uint8_t reg);
static void write8(uint8_t reg, uint32_t value);
static void ErrorHandler(void);

static uint8_t read8(uint8_t reg) {
	uint8_t commandByte = TCS34725_COMMAND_BIT | reg ;
	uint8_t retVal = 0 ;

	ret = HAL_I2C_Mem_Read(&hi2c1,(uint16_t)(TCS34725_I2C_ADDR<<1),commandByte,I2C_MEMADD_SIZE_8BIT,&retVal,1,1);
	if (ret!=HAL_OK) {
	    ErrorHandler();
	}
	return retVal;
}

static void write8(uint8_t reg, uint32_t value) {
	uint8_t cmdbuf[2] = {TCS34725_COMMAND_BIT|reg,value};
	ret = HAL_I2C_Master_Transmit(&hi2c1,TCS34725_I2C_ADDR<<1,cmdbuf,2,1);
	if (ret!=HAL_OK) {
	    ErrorHandler();
	}
}

void get_crgb(uint16_t* c,uint16_t* r,uint16_t* g,uint16_t* b) {
	uint8_t buf[8];
	uint8_t commandByte = TCS34725_COMMAND_BIT | TCS34725_COLOR_START ;
	ret = HAL_I2C_Mem_Read(&hi2c1,(uint16_t)(0x29<<1),commandByte,I2C_MEMADD_SIZE_8BIT,buf,8,100);
	if (ret!=HAL_OK) {
		    ErrorHandler();
		}
    *c = buf[1]<<8|buf[0];
	*r = buf[3]<<8|buf[2];
	*g = buf[5]<<8|buf[4];
	*b = buf[7]<<8|buf[6];

}

void init_tcs34725(void) {
	uint8_t cmdbuf[2] = {0x80,0x01};
	HAL_I2C_Master_Transmit(&hi2c1,TCS34725_I2C_ADDR<<1,cmdbuf,2,1);
	HAL_Delay(3);
	cmdbuf[0] = 0x80 | TCS34725_ENABLE;
	cmdbuf[1] = TCS34725_ENABLE_PON|TCS34725_ENABLE_AEN;
	HAL_I2C_Master_Transmit(&hi2c1,TCS34725_I2C_ADDR<<1,cmdbuf,2,1);
	cmdbuf[0] = 0x80 | TCS34725_ATIME;
	cmdbuf[1] = TCS34725_INTEGRATIONTIME_2_4MS;
	HAL_I2C_Master_Transmit(&hi2c1,TCS34725_I2C_ADDR<<1,cmdbuf,2,1);
	cmdbuf[0] = 0x80 |TCS34725_CONTROL;
	cmdbuf[1] = TCS34725_GAIN_1X;
	HAL_I2C_Master_Transmit(&hi2c1,TCS34725_I2C_ADDR<<1,cmdbuf,2,1);
}

static void ErrorHandler(void){
	HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
}
