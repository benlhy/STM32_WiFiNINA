/*
 * wifinina.h
 *
 *  Created on: Aug 4, 2020
 *      Author: ben
 */

#ifndef INC_UTILITY_WIFININA_H_
#define INC_UTILITY_WIFININA_H_

#include "stm32f7xx_hal.h"
//#include "cmsis_os.h" // uncommment to work with FreeRTOS
#include "main.h"

#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET

#define millis() HAL_GetTick()

#define boolean bool

#define INPUT GPIO_MODE_INPUT
#define AINPUT GPIO_MODE_ANALOG
#define OUTPUT GPIO_MODE_OUTPUT_PP
#define NOPULL GPIO_NOPULL
#define PULLUP GPIO_PULLUP
#define PULLDOWN GPIO_PULLDOWN

#define WIFININA_SLAVEGPIO0_PORT GPIOA
#define WIFININA_SLAVEREADY_PORT GPIOF
#define WIFININA_SLAVESELECT_PORT GPIOD
#define WIFININA_SLAVERESET_PORT GPIOF


extern int16_t WIFININA_SLAVESELECT, WIFININA_SLAVEREADY, WIFININA_SLAVERESET,
    WIFININA_SLAVEGPIO0;

void pinMode(GPIO_TypeDef* port, uint32_t pin, uint32_t mode, uint32_t pull);
void delay(uint32_t val);
GPIO_PinState digitalRead(GPIO_TypeDef* port,  uint32_t pin);
void digitalWrite(GPIO_TypeDef* port,  uint32_t pin, GPIO_PinState state);
char SPITransfer(volatile char data);

#endif /* INC_UTILITY_WIFININA_H_ */
