/*********************************************************************************
JJ_fly by TIVA
文件名：US100.c
描述：US100数据读取与处理
作者：卢华歆
时间：2016.4
**********************************************************************************/
#include "uarts.h"
#include "mymath.h"
#include "US100.h"
#include "imu_new.h"
float dis;
float US100_Alt;
float US100_Alt_old;
void US_100_init()//串口初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//使能uart0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//开外设使能
	GPIOPinConfigure(GPIO_PC6_U3RX);
	GPIOPinConfigure(GPIO_PC7_U3TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE,GPIO_PIN_6|GPIO_PIN_7);//使能相应IO
	//uart0,波特率115200,8位数据位1位停止位
	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTFIFOLevelSet(UART3_BASE,UART_FIFO_TX7_8,UART_FIFO_RX6_8);//发送FIFO达到7/8时中断，接收FIFO达到6/8时中断
    //IntMasterEnable();//开启总中断
    //IntEnable(INT_UART3_SNOWFLAKE);//开启uart0中断
    //UARTIntEnable(UART3_BASE, UART_INT_RX);//仅开启RX中断
}
void US_100_trig()//触发信号
{
	UARTCharPutNonBlocking(UART3_BASE, 0X55);
}
void US_100_get()//读取返回距离值
{
	unsigned char temp_H,temp_L;
	temp_H=UARTCharGetNonBlocking(UART3_BASE);
	temp_L=UARTCharGetNonBlocking(UART3_BASE);
	//超声波的感应角度是15°当飞机角度很大使不进行高度读取
	if(Q_ANGLE_X<13.0f&&Q_ANGLE_X>-13.0f&&Q_ANGLE_Y<13.0f&&Q_ANGLE_Y>-13.0f)
	{
		dis=(float)(temp_H*256+temp_L)/100;
		US100_Alt = Moving_Median(2,4,dis);//移动中位值滤波，深度不宜太大
		if(US100_Alt-US100_Alt_old>5.0f||US100_Alt-US100_Alt_old<-5.0f)//滤掉一些毛刺（高度突变）
		{
			US100_Alt=US100_Alt_old;
		}
		US100_Alt_old = US100_Alt;//保存历史数据
	}
}

