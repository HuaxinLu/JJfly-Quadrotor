/*********************************************************************************
JJ_fly by TIVA
文件名：SPI.h
描述：硬件SPI
作者：卢华歆
时间：2015.10
**********************************************************************************/
#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#define GPIO_PA2_SSI0CLK        0x00000802
#define GPIO_PA3_SSI0FSS        0x00000C02
#define GPIO_PA4_SSI0RX         0x00001002
#define GPIO_PA5_SSI0TX         0x00001402
#define SSI_BitRate 800000
void SPI_TM4C123_init(void);
uint8_t SPIReadStatus(uint8_t reg);
uint8_t SPI_Write_Buf(uint8_t WriteReg, uint8_t *pBuf, uint8_t bytes);
uint8_t SPI_Write_register(uint8_t WriteReg,uint8_t Value);
uint8_t SPI_RW(uint8_t value);
#endif /* SPI_H_ */
