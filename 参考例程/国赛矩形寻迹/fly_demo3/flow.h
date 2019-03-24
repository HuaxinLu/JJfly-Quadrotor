/*********************************************************************************
JJ_fly by TIVA
文件名：flow.h
描述：ADNS3080数据读取与处理
作者：卢华歆
时间：2016.4
**********************************************************************************/
#ifndef FLOW_H_
#define FLOW_H_
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
extern char flow_flag;
void flow_init();
void flow_filter();
void k60_init();
extern char vx,vy;
extern unsigned char X1,Y1,X2,Y2;
#endif /* FLOW_H_ */
