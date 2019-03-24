/*********************************************************************************
JJ_fly by TIVA
文件名：hmc5883.c
描述：hmc5883数据读取与处理
作者：卢华歆
时间：2015.10
**********************************************************************************/
#include "hmc5883.h"
#include "IIC.h"
#include "mpu6050.h"
int hmc5883_x,hmc5883_y,hmc5883_z;//磁场初始值
int HMC_AVG_X,HMC_AVG_Y,HMC_AVG_Z;//平均值滤波后的磁力值
int HMC_X_BUF[FILTER_NUM],HMC_Y_BUF[FILTER_NUM],HMC_Z_BUF[FILTER_NUM];//磁力计滑动滤波窗口

void hmc5883_init()//初始化传感器
{
	SLAVE_ADDRESS=0x1E;
	i2c_write(0x00,0x18);//正常测量模式，数据输出速率75hz，无采样平均
    i2c_write(0x01, 0x20);//增益选择
    i2c_write(0x02,0x00);//连续测量模式
}
void hmc5883_read()//读取传感器数据
{
	long int temp1=0,temp2=0,temp3=0;
	static unsigned char filter_cnt_=0;
	unsigned char i;
	SLAVE_ADDRESS=0x1E;

	hmc5883_x=Get16Bit(0x03);//读取三轴传感器磁场值
	hmc5883_y=Get16Bit(0x05);
	hmc5883_z=Get16Bit(0x07);

	HMC_X_BUF[filter_cnt_] = hmc5883_x;//更新滑动窗口数组
	HMC_Y_BUF[filter_cnt_] = hmc5883_y;
	HMC_Z_BUF[filter_cnt_] = hmc5883_z;

	for(i=0;i<FILTER_NUM;i++)
	{
	  temp1 += HMC_X_BUF[i];
	  temp2 += HMC_Y_BUF[i];
	  temp3 += HMC_Z_BUF[i];
	}

	HMC_AVG_X = temp1 / FILTER_NUM;
	HMC_AVG_Y = temp2 / FILTER_NUM;
	HMC_AVG_Z = temp3 / FILTER_NUM;

	filter_cnt_++;
	if(filter_cnt_==FILTER_NUM)
	{
		filter_cnt_=0;
	}
}
