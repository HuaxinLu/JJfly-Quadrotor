/*********************************************************************************
JJ_fly by TIVA
文件名：uarts.h
描述：串口发送与接收，含匿名新版旧版上位机的通讯协议，接收采用中断，调试用
作者：卢华歆
时间：2015.10
**********************************************************************************/

#ifndef UARTS_H_
#define UARTS_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA1_U0TX           0x00000401

void uart_init();
void  UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void ANO_DT_Send_Status();
void ANO_DT_Send_RCData();
void ANO_DT_Send_MotoPWM();
void UART_Send_AF(void);
void UART_Send_AE(void);
void  ANO_DT_Send_Speed();
extern unsigned char UART_TXDATA[35];
extern unsigned char rx_data[23];
#endif /* UARTS_H_ */
