/*********************************************************************************
JJ_fly by TIVA
文件名：init.c
描述：初始化及自检程序
作者：卢华歆
时间：2015.11
**********************************************************************************/
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
#include "imu_new.h"
#include "mymath.h"
#include "US100.h"
#include "flow.h"
unsigned char _time_flag_1;
unsigned char _time_flag_2;
int check_count=100;
int check_count2=400;
int check_end;
int check_aspeed_end;
int yaw_check,yaw_check_old;
long int offset_gyro_z,offset_gyro_y,offset_gyro_x;
long int offset_acc_z,offset_acc_y,offset_acc_x;
float G;
void all_init()
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//外部16M晶振
	init_first();//自检前初始化
	check();//自检
	init_last();//自检后初始化
}
void init_first()//自检前初始化
{
	capture_init();//初始化捕获定时器，负责计算遥控数据
	uart_init();//初始化串口
	i2c_init();//初始化硬件IIC
	pwm_init();//pwm初始化
	mpu6050_init();//初始化mpu6050
	US_100_init();//初始化超声波
	while(i2c_read(WHO_AM_I)!=0x68);//校验6050地址
}
void init_last()//自检后初始化
{
	control_init();//pid参数初始化
	timer_init();//初始化主定时器，负责任务调度
	//flow_init();
	k60_init();
}
void check()//自检
{
	//辅助通道回中，自检开关打开
	while(CH_3>CH_3_TEST+5||CH_3<CH_3_TEST-5)//辅助开关打到中间，容错10
	{
		get_rc_value();//读取遥控器
	}
	//自检定时器打开
	delay_ms(100);
	check_start();
	//油门值归零，其他摇杆回中,容错10
	while(CH_1<CH_1_MIN-5||CH_1>CH_1_MIN+5||CH_2<CH_2_MID-5|| CH_2>CH_2_MID+5||CH_5<CH_5_MID-5||CH_5>CH_5_MID+5||CH_6<CH_6_MID-5||CH_6>CH_6_MID+5)
	{
		get_rc_value();//读取遥控器
	}
	//等待自检完成
	while(check_end != 1);
}

void check_6050_offest()//6050自检补偿数据处理
{
	check_count--;//自检计数-1
	//////////////////求得yaw漂移并抑制//////////////////
	offset_gyro_z += MPU6050_GYRO_LAST_Z;
	offset_gyro_y += MPU6050_GYRO_LAST_Y;
	offset_gyro_x += MPU6050_GYRO_LAST_X;
	offset_acc_z += MPU6050_ACC_LAST_Z;
	offset_acc_y += MPU6050_ACC_LAST_Y;
	offset_acc_x += MPU6050_ACC_LAST_X;
	if(check_count==0)//角速度自检完成
	{
		GYRO_OFFSET_Z = offset_gyro_z / 100;//求得z补偿
		GYRO_OFFSET_Y = offset_gyro_y / 100;//求得y补偿
		GYRO_OFFSET_X = offset_gyro_x / 100;//求得x补偿
		ACC_OFFSET_Z = offset_acc_z / 100;
		ACC_OFFSET_Y = offset_acc_y / 100;
		ACC_OFFSET_X = offset_acc_x / 100;
		G= my_sqrt(ACC_OFFSET_X*ACC_OFFSET_X + ACC_OFFSET_Y*ACC_OFFSET_Y + ACC_OFFSET_Z*ACC_OFFSET_Z);;
		check_aspeed_end = 1;//标志位
		//关闭自检定时器
		//TimerIntDisable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
	}
}
void check_angle_offest()//角度自检补偿数据处理
{
	if(check_aspeed_end == 1)//确定角速度校准完成
	{
		check_count2--;//自检计数-1
		if(check_count2<100)
		{
			//////////////////角度去零点//////////////////
			offset_angle_x += Q_ANGLE_X;
			offset_angle_y += Q_ANGLE_Y;
			if(check_count2==0)//角度自检完成
			{
				offset_angle_y = offset_angle_y / 100;//求得y补偿
				offset_angle_x = offset_angle_x / 100;//求得x补偿
				check_end = 1;//标志位
				//关闭自检定时器
				TimerIntDisable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
			}
		}

	}
}
void check_start()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);//使能TIMER0
	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PERIODIC);//周期性计数模式
	TimerLoadSet(TIMER0_BASE, TIMER_B,SysCtlClockGet() / 400 - 1);//计数频率400HZ
	IntEnable(INT_TIMER0B_SNOWFLAKE);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);//TIMEOUT标志位触发中断
    IntMasterEnable();//总中断
    TimerEnable(TIMER0_BASE, TIMER_B);//TIMER0A开始计数，当计数值等于TimerLoadSet，触发中断
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}
void Timer0BIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);//清除标志位
    _time_flag_1++;//时间标志1
    //////////////2.5ms任务//////////////////
    Prepare_Data();//姿态传感器数据读取及简单处理
    get_rc_value();//读取遥控器
    /////////////////////////////////////
    if(_time_flag_1==2)
    {
    	_time_flag_1=0;
        //////////////5ms任务//////////////////
    	Get_Attitude();//进行姿态解算
    	check_6050_offest();
    	check_angle_offest();
        ///////////////////////////////////
    }
}

