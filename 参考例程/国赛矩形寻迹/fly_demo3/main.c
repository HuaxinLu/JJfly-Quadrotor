/*********************************************************************************
JJ_fly by TIVA
文件名：main.c
描述：初始化及主循环
作者：卢华歆
时间：2015.10
**********************************************************************************/
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

#include "display_oled.h"
#include "IIC.h"
#include "mpu6050.h"
#include "uarts.h"
#include "timers.h"
#include "hmc5883.h"
#include "nrf2401.h"
#include "capture.h"
#include "PWM.h"
#include "control.h"
#include "init.h"
void main(void)
{
	delay_ms(500);
	all_init();//初始化全部
    while(1)
    {
    	if(UART_Send_flag==1)
    	{
    		//get_rc_value();
    		ANO_DT_Send_RCData();
    		ANO_DT_Send_Status();
    		ANO_DT_Send_MotoPWM();
    		//ANO_DT_Send_Speed();
    		UART_Send_flag=0;
    	}
    }
}



