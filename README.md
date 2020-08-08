# STM32

This repo uses the Nucleo STM32F767ZI as a starting point. Please note that you need to define `SSID` and `PASSWORD` in a `secret.h` file to use the WiFi code.

To convert the WiFiNINA code for usage in other frameworks, change the code in the following places:

1. `utility/wifinina.h` - the pins and the functions have to be changed to the ones suitable for the framework and to match the pins that you are using.

# Pinouts for Nucleo-F767ZI

![](/images/header1.JPG)
![](/images/header2.JPG)

# Configuration
* I2C (I2C1): PB8 (SCL), PB9 (SDA)
* SPI (SPI1): SCK - PA5, MOSI - PA7 (master transmit only), CS - PD14 (manually toggle)
* SPI (SPI1): SCK - PA5, MOSI - PA7, MISO - PA6, CS - PD14, READY - PF13, RESET - PE11


# Add on Boards

TCS34725 - added .h and .c files

## Proving Fields Toolkit
* MCP23008 (I2C, GPIO expander) - added .h and .c files
* MCP4922 (SPI, DAC) - added .h and .c files

## ESP32 Airlift
* ESP32 (SPI, WiFi coprocessor) - added .h and .cpp files

