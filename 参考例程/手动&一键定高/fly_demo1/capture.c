/*********************************************************************************
JJ_fly by TIVA
文件名：capture.c
描述：捕获遥控器接收机发出的PWM，最多可捕获六路
作者：卢华歆
时间：2015.10
**********************************************************************************/
#include "capture.h"
unsigned char PPM_1,PPM_2,PPM_3,PPM_4,PPM_5,PPM_6;//标志，判断上升沿or下降沿

uint32_t PPM_1_TIME_1,PPM_1_TIME_2;
uint32_t PPM_2_TIME_1,PPM_2_TIME_2;
uint32_t PPM_3_TIME_1,PPM_3_TIME_2;
uint32_t PPM_4_TIME_1,PPM_4_TIME_2;
uint32_t PPM_5_TIME_1,PPM_5_TIME_2;
uint32_t PPM_6_TIME_1,PPM_6_TIME_2;
uint32_t CH_5_temp;
uint16_t CH_1,CH_2,CH_3,CH_4,CH_5,CH_6;//转化后用于运算的通道值
uint32_t test;
unsigned char CH1_flag,CH2_flag,CH3_flag,CH4_flag,CH5_flag,CH6_flag;
unsigned long CH1_AVG,CH2_AVG,CH3_AVG,CH4_AVG,CH5_AVG,CH6_AVG;

void capture_init()//初始化捕获引脚
{
    //////////////////开启定时器//////////////////////////////
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    //////////////////////////////////////////////////////////
    //TimerLoadSet(TIMER1_BASE, TIMER_A,SysCtlClockGet() / 800 - 1);//计数频率400HZ
    TimerPrescaleSet(TIMER1_BASE, TIMER_A, 100);
    /////////////////使能捕获IO///////////////////////////////
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    /////////////////////////////////////////////////////////
    SysCtlDelay(2);
    ////////////////配置相应IO///////////////////////////////
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3);

    GPIOPinConfigure(GPIO_PB4_T1CCP0);
    GPIOPinConfigure(GPIO_PB5_T1CCP1);
    GPIOPinConfigure(GPIO_PB0_T2CCP0);
    GPIOPinConfigure(GPIO_PB1_T2CCP1);//
    GPIOPinConfigure(GPIO_PB2_T3CCP0);
    GPIOPinConfigure(GPIO_PB3_T3CCP1);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    //////////////////////////////////////////////////////////
    ////////////////扩展捕获位/////////////////////////////
    TimerPrescaleSet(TIMER1_BASE,TIMER_A,255);
    TimerPrescaleSet(TIMER1_BASE,TIMER_B,255);
    TimerPrescaleSet(TIMER2_BASE,TIMER_A,255);
    TimerPrescaleSet(TIMER2_BASE,TIMER_B,255);
    TimerPrescaleSet(TIMER3_BASE,TIMER_A,255);
    TimerPrescaleSet(TIMER3_BASE,TIMER_B,255);
    //////////////////////////////////////////////////////////
    /////////////////设置捕获模式////////////////////////////
    TimerConfigure(TIMER1_BASE,TIMER_CFG_B_CAP_TIME_UP|TIMER_CFG_A_CAP_TIME_UP|TIMER_CFG_SPLIT_PAIR);//计时捕获模式，上升沿捕获
    TimerConfigure(TIMER2_BASE,TIMER_CFG_B_CAP_TIME_UP|TIMER_CFG_A_CAP_TIME_UP|TIMER_CFG_SPLIT_PAIR);//计时捕获模式，上升沿捕获
    TimerConfigure(TIMER3_BASE,TIMER_CFG_B_CAP_TIME_UP|TIMER_CFG_A_CAP_TIME_UP|TIMER_CFG_SPLIT_PAIR);//计时捕获模式，上升沿捕获

    TimerControlEvent(TIMER1_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
    TimerControlEvent(TIMER1_BASE,TIMER_B,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
    TimerControlEvent(TIMER2_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
    TimerControlEvent(TIMER2_BASE,TIMER_B,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
    TimerControlEvent(TIMER3_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
    TimerControlEvent(TIMER3_BASE,TIMER_B,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
    ///////////////////////////////////////////////////////////
    //TimerPrescaleSet(TIMER1_BASE,TIMER_A, 128) ;
    //TimerPrescaleSet(TIMER2_BASE,TIMER_A, 128) ;
    //TimerPrescaleSet(TIMER3_BASE,TIMER_A, 128) ;
    /////////////////使能中断/////////////////////////////////
    IntEnable(INT_TIMER1A_SNOWFLAKE);//使能TIMER0A
    IntEnable(INT_TIMER1B_SNOWFLAKE);//使能TIMER0A
    IntEnable(INT_TIMER2A_SNOWFLAKE);//使能TIMER0A
    IntEnable(INT_TIMER2B_SNOWFLAKE);//使能TIMER0A
    IntEnable(INT_TIMER3A_SNOWFLAKE);//使能TIMER0A
    IntEnable(INT_TIMER3B_SNOWFLAKE);//使能TIMER0A

    TimerIntEnable(TIMER1_BASE, TIMER_CAPA_EVENT);//定时器A捕获事件触发中断
    TimerIntEnable(TIMER1_BASE, TIMER_CAPB_EVENT);//定时器A捕获事件触发中断
    TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT);//定时器A捕获事件触发中断
    TimerIntEnable(TIMER3_BASE, TIMER_CAPB_EVENT);//定时器A捕获事件触发中断
    TimerIntEnable(TIMER3_BASE, TIMER_CAPA_EVENT);//定时器A捕获事件触发中断
    TimerIntEnable(TIMER3_BASE, TIMER_CAPB_EVENT);//定时器A捕获事件触发中断

    IntMasterEnable();

    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_B);
    TimerEnable(TIMER2_BASE, TIMER_A);
    TimerEnable(TIMER2_BASE, TIMER_B);
    TimerEnable(TIMER3_BASE, TIMER_A);
    TimerEnable(TIMER3_BASE, TIMER_B);
    /////////////////////////////////////////////////////////////////
    /////////////////数据初始化///////////////////////////////
    PPM_1=0;
    PPM_2=0;
    PPM_3=0;
    PPM_4=0;
    PPM_5=0;
    PPM_6=0;
    /////////////////////////////////////////////////////////////
}
/////////////////捕获中断///////////////////////////////
void Timer1AIntHandler(void)
{
	uint32_t temp;
	TimerIntClear(TIMER1_BASE,TIMER_CAPA_EVENT);
	test=TimerPrescaleGet(TIMER1_BASE,TIMER_A) ;
	if(PPM_1==0)
	{
		TimerControlEvent(TIMER1_BASE,TIMER_A,TIMER_EVENT_NEG_EDGE);//捕获模式，A定时器，下降沿捕获
		PPM_1_TIME_1=TimerValueGet(TIMER1_BASE,TIMER_A);
		PPM_1=1;
	}
	else
	{
		TimerControlEvent(TIMER1_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
		temp=TimerValueGet(TIMER1_BASE,TIMER_A);
		//test=TimerPrescaleGet(TIMER1_BASE,TIMER_A) ;
		if(temp>PPM_1_TIME_1)
		{
			PPM_1_TIME_2=temp-PPM_1_TIME_1;
		}
		else
		{
			PPM_1_TIME_2=0xffffff-PPM_1_TIME_1+temp;

		}
		PPM_1=0;
	}
}
void Timer1BIntHandler(void)
{
	uint32_t temp;
	TimerIntClear(TIMER1_BASE,TIMER_CAPB_EVENT);
	if(PPM_2==0)
	{
		TimerControlEvent(TIMER1_BASE,TIMER_B,TIMER_EVENT_NEG_EDGE);//捕获模式，A定时器，下降沿捕获
		PPM_2_TIME_1=TimerValueGet(TIMER1_BASE,TIMER_B);
		PPM_2=1;
	}
	else
	{
		TimerControlEvent(TIMER1_BASE,TIMER_B,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
		temp=TimerValueGet(TIMER1_BASE,TIMER_B);
		if(temp>PPM_2_TIME_1)
		{
			PPM_2_TIME_2=temp-PPM_2_TIME_1;
		}
		else
		{
			PPM_2_TIME_2=0xffffff-PPM_2_TIME_1+temp;

		}
		PPM_2=0;
	}
}

void Timer2AIntHandler(void)
{
	uint32_t temp;
	TimerIntClear(TIMER2_BASE,TIMER_CAPA_EVENT);
	if(PPM_3==0)
	{
		TimerControlEvent(TIMER2_BASE,TIMER_A,TIMER_EVENT_NEG_EDGE);//捕获模式，A定时器，下降沿捕获
		PPM_3_TIME_1=TimerValueGet(TIMER2_BASE,TIMER_A);
		PPM_3=1;
	}
	else
	{
		TimerControlEvent(TIMER2_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
		temp=TimerValueGet(TIMER2_BASE,TIMER_A);
		if(temp>PPM_2_TIME_1)
		{
			PPM_3_TIME_2=temp-PPM_3_TIME_1;
		}
		else
		{
			PPM_3_TIME_2=0xffffff-PPM_3_TIME_1+temp;

		}
		PPM_3=0;
	}
}

void Timer2BIntHandler(void)
{
	uint32_t temp;
	TimerIntClear(TIMER2_BASE,TIMER_CAPB_EVENT);
	if(PPM_4==0)
	{
		TimerControlEvent(TIMER2_BASE,TIMER_B,TIMER_EVENT_NEG_EDGE);//捕获模式，A定时器，下降沿捕获
		PPM_4_TIME_1=TimerValueGet(TIMER2_BASE,TIMER_B);
		PPM_4=1;
	}
	else
	{
		TimerControlEvent(TIMER2_BASE,TIMER_B,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
		temp=TimerValueGet(TIMER2_BASE,TIMER_B);
		if(temp>PPM_4_TIME_1)
		{
			PPM_4_TIME_2=temp-PPM_4_TIME_1;
		}
		else
		{
			PPM_4_TIME_2=0xffffff-PPM_4_TIME_1+temp;

		}
		PPM_4=0;
	}
}
void Timer3AIntHandler(void)
{
	uint32_t temp;
	TimerIntClear(TIMER3_BASE,TIMER_CAPA_EVENT);
	if(PPM_5==0)
	{
		TimerControlEvent(TIMER3_BASE,TIMER_A,TIMER_EVENT_NEG_EDGE);//捕获模式，A定时器，下降沿捕获
		PPM_5_TIME_1=TimerValueGet(TIMER3_BASE,TIMER_A);
		if(PPM_5_TIME_1>0xffffff)
		{
			PPM_5_TIME_1=PPM_5_TIME_1;
		}
		PPM_5=1;
	}
	else
	{
		TimerControlEvent(TIMER3_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
		temp=TimerValueGet(TIMER3_BASE,TIMER_A);
		if(temp>PPM_5_TIME_1)
		{
			PPM_5_TIME_2=temp-PPM_5_TIME_1;
		}
		else
		{
			PPM_5_TIME_2=0xffffff-PPM_5_TIME_1+temp;

		}
		PPM_5=0;
	}
}
void Timer3BIntHandler(void)
{
	uint32_t temp;
	TimerIntClear(TIMER3_BASE,TIMER_CAPB_EVENT);
	if(PPM_6==0)
	{
		TimerControlEvent(TIMER3_BASE,TIMER_B,TIMER_EVENT_NEG_EDGE);//捕获模式，A定时器，下降沿捕获
		PPM_6_TIME_1=TimerValueGet(TIMER3_BASE,TIMER_B);
		PPM_6=1;
	}
	else
	{
		TimerControlEvent(TIMER3_BASE,TIMER_B,TIMER_EVENT_POS_EDGE);//捕获模式，A定时器，上升沿捕获
		temp=TimerValueGet(TIMER3_BASE,TIMER_B);
		if(temp>PPM_2_TIME_1)
		{
			PPM_6_TIME_2=temp-PPM_6_TIME_1;
		}
		else
		{
			PPM_6_TIME_2=0xffffff-PPM_6_TIME_1+temp;

		}
		PPM_6=0;
	}
}
void get_rc_value()//处理通道值
{
	if(PPM_1_TIME_2>79998&&PPM_1_TIME_2<160002)
	{
		CH_1=(PPM_1_TIME_2/800);//油门THR  1000--1960
		//CH1_flag++;
		if(CH1_flag==5)
		{
			CH_1=CH1_AVG/5;
			CH1_AVG=0;
			CH1_flag=0;
		}
	}
	if(PPM_2_TIME_2>79998&&PPM_2_TIME_2<160002)
	{
		CH2_AVG+=(PPM_2_TIME_2/800);//偏航YAW  1000-1476-1958
		CH2_flag++;
		if(CH2_flag==5)
		{
			CH_2=CH2_AVG/5;
			CH2_AVG=0;
			CH2_flag=0;
		}
	}
	if(PPM_3_TIME_2>71998&&PPM_3_TIME_2<200002)
	{
		/*CH3_AVG+=(PPM_3_TIME_2/800);//辅助 996-1476-1956
		CH3_flag++;
		if(CH3_flag==5)
		{
			CH_3=CH3_AVG/5;
			CH3_AVG=0;
			CH3_flag=0;
		}*/
		CH_3=(PPM_3_TIME_2/800);
	}
	if(PPM_4_TIME_2>71998&&PPM_4_TIME_2<200002)
	{
		CH4_AVG+=(PPM_4_TIME_2/800);//辅助 996-1476-1956
		CH4_flag++;
		if(CH4_flag==5)
		{
			CH_4=CH4_AVG/5;
			CH4_AVG=0;
			CH4_flag=0;
		}
	}
	if(PPM_5_TIME_2>79998&&PPM_5_TIME_2<160002)
	{
		CH5_AVG+=(PPM_5_TIME_2/800);//翻滚ROL  1000-1478-1958
		CH5_flag++;
		if(CH5_flag==5)
		{
			CH_5=CH5_AVG/5;
			CH5_AVG=0;
			CH5_flag=0;
		}
	}
	if(PPM_6_TIME_2>79998&&PPM_6_TIME_2<160002)
	{
		CH6_AVG+=(PPM_6_TIME_2/800);//俯仰PIT  1000-1480-1960.
		CH6_flag++;
		if(CH6_flag==5)
		{
			CH_6=CH6_AVG/5;
			CH6_AVG=0;
			CH6_flag=0;
		}
	}
}
