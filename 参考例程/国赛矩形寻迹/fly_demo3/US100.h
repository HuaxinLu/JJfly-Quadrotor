/*********************************************************************************
JJ_fly by TIVA
文件名：US100.h
描述：US100数据读取与处理
作者：卢华歆
时间：2016.4
**********************************************************************************/

#ifndef US100_H_
#define US100_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "uarts.h"
#define GPIO_PC6_U3RX           0x00021801
#define GPIO_PC7_U3TX           0x00021C01
void US_100_init();
void US_100_trig();
void US_100_get();
extern float dis;
extern float US100_Alt;
#endif /* US100_H_ */
