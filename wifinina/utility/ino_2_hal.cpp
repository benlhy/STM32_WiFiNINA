#include "ino_2_hal.h"

#define INPUT GPIO_MODE_INPUT
#define AINPUT GPIO_MODE_ANALOG
#define OUTPUT GPIO_MODE_OUTPUT_PP
#define NOPULL GPIO_NOPULL
#define PULLUP GPIO_PULLUP
#define PULLDOWN GPIO_PULLDOWN



void pinMode(uint8_t port, uint8_t pin, uint8_t mode, uint8_t pull) {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = pull;
  HAL_GPIO_Init(port, &GPIO_InitStruct)
}
