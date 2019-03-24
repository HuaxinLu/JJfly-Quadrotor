/*********************************************************************************
JJ_fly by TIVA
文件名：hmc5883.h
描述：hmc5883数据读取与处理
作者：卢华歆
时间：2015.10
**********************************************************************************/

#ifndef HMC5883_H_
#define HMC5883_H_

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
#include "driverlib/timer.h"


extern int hmc5883_x,hmc5883_y,hmc5883_z;
extern int HMC_AVG_X,HMC_AVG_Y,HMC_AVG_Z;//平均值滤波后的磁力值
void hmc5883_init();
void hmc5883_read();
#endif /* HMC5883_H_ */
