/*********************************************************************************
JJ_fly by TIVA
文件名：timers.c
描述：任务调度
作者：卢华歆
时间：2015.10
**********************************************************************************/
#include "timers.h"
#include "mpu6050.h"
#include "nrf2401.h"
#include "imu_new.h"
#include "hmc5883.h"
#include "capture.h"
#include "control.h"
#include "US100.h"
#include "flow.h"
unsigned char flag=0;
unsigned char time_flag_1=0;
unsigned char time_flag_2=0;
unsigned char time_flag_3=0;
unsigned char time_flag_4=0;
unsigned char time_flag_5=0;
unsigned char UART_Send_flag=0;
unsigned char height_ready;
unsigned char height_control;
//定时器初始化，使用timer0A，连续计数，溢出中断
void timer_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);//使能TIMER0
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);//周期性计数模式
	TimerLoadSet(TIMER0_BASE, TIMER_A,SysCtlClockGet() / 400 - 1);//计数频率400HZ
	IntEnable(INT_TIMER0A_SNOWFLAKE);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);//TIMEOUT标志位触发中断
    IntMasterEnable();//总中断
    TimerEnable(TIMER0_BASE, TIMER_A);//TIMER0A开始计数，当计数值等于TimerLoadSet，触发中断
}
//timer0中断处理函数，2.5ms进入一次
void Timer0AIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);//清除标志位
    time_flag_1++;//时间标志1
    time_flag_2++;//时间标志2
    time_flag_3++;//时间标志3
    time_flag_4++;//时间标志4
    time_flag_5++;//时间标志5
    //////////////2.5ms任务//////////////////
    get_rc_value();
    Prepare_Data();//姿态传感器数据读取及简单处理
    //hmc5883_read();
    //SPI_Write_register(WRITE_REG+STATUS,0xff);//清除nrf2401的状态寄存器，准备下一次发射
    /////////////////////////////////////
    if(time_flag_1==2)
    {
    	time_flag_1=0;
        //////////////5ms任务//////////////////
    	//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,0);
    	Get_Attitude();//进行姿态解算
    	control();
    	//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,0xff);
        ///////////////////////////////////
    }
    if(time_flag_2==3)
    {
    	time_flag_2=0;
        //////////////7.5ms任务//////////////////
    	UART_Send_flag++;//开启数据发送标志位
    	//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,0);
    	//nrf2401_send_RCData();
    	//nrf2401_send_RCData();
	    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,0xff);
        //SysCtlDelay(10000);
        //SPI_Write_register(WRITE_REG+STATUS,0xff);
        ///////////////////////////////////
    }
    if(time_flag_3==15)//30ms任务
    {
    	if(height_ready==0)
    	{
    		US_100_trig();
    		height_ready=1;
    	}
    	else if(height_ready==1)
    	{
    		US_100_get();
    		height_ready=0;
    		height_control++;
    		if(height_control==2)
    		{
    			height_control=0;
    			control_height();
    		}
    	}
    	time_flag_3=0;
    }
    if(time_flag_4==80)//100ms任务
    {
    	//control_point_1();
    	time_flag_4=0;
    }
    if(time_flag_5==120)//500ms任务
    {
    	land();
    	time_flag_5=0;
    }
}

