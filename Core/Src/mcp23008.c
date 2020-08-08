/*
 * mcp23008.c
 *
 *  Created on: Aug 1, 2020
 *      Author: ben
 */
#include  "mcp23008.h"


HAL_StatusTypeDef ret;
uint8_t read8(uint8_t reg);
void write8(uint8_t reg, uint32_t value);
void ErrorHandler(void);

uint8_t read8(uint8_t reg) {
	//uint8_t commandByte = TCS34725_COMMAND_BIT | reg ;
	uint8_t retVal = 0 ;
	ret = HAL_I2C_Mem_Read(&hi2c1,MCP23008_I2C_ADDR<<1,reg,I2C_MEMADD_SIZE_8BIT,&retVal,1,1);
	if (ret!=HAL_OK) {
	    ErrorHandler();
	}
	return retVal;
}

void write8(uint8_t reg, uint32_t value) {
	uint8_t cmdbuf[2] = {reg , value};
	ret = HAL_I2C_Master_Transmit(&hi2c1,MCP23008_I2C_ADDR<<1,cmdbuf,2,1);
	if (ret!=HAL_OK) {
	    ErrorHandler();
	}
}

void ErrorHandler(void) {
	HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
}

void init_mcp23008(void) {
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
	write8(MCP23008_IODIR, 0x00); // all outputs
}

void digitalWrite_mcp23008(uint8_t pin, uint8_t val) {
	uint8_t gpio = read8(MCP23008_GPIO);
	if (val==1) {
		gpio = 1<<pin | gpio;
	}
	else {
		gpio = ~(1<<pin) & gpio;
	}
	write8(MCP23008_GPIO,gpio);

}

void pinMode_mcp23008(uint8_t pin, uint8_t dir) {

}
