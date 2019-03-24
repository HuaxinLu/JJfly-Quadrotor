/*********************************************************************************
JJ_fly by TIVA
文件名：timers.h
描述：任务调度
作者：卢华歆
时间：2015.10
**********************************************************************************/
#ifndef TIMERS_H_
#define TIMERS_H_
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

extern unsigned char flag;
extern unsigned char time_flag_1;
extern unsigned char time_flag_2;
extern unsigned char UART_Send_flag;

extern unsigned char game_count;
extern unsigned char error_flag;
extern unsigned char game_time_1;

void timer_init();
void Timer0AIntHandler(void);

#endif /* TIMERS_H_ */
