/*********************************************************************************
JJ_fly by TIVA
文件名：mpu6050.c
描述：mpu6050数据读取与处理，参考匿名旧版程序
作者：卢华歆
时间：2015.10
**********************************************************************************/
#include "mpu6050.h"
#include "IIC.h"
#include "math.h"
#include "display_oled.h"
#include "hmc5883.h"
#include "init.h"
//陀螺仪加速度计的补偿量
int ACC_OFFSET_X,ACC_OFFSET_Y,ACC_OFFSET_Z;
int GYRO_OFFSET_X,GYRO_OFFSET_Y,GYRO_OFFSET_Z;
//陀螺仪加速度计实时值，带修正
int MPU6050_ACC_LAST_X,MPU6050_ACC_LAST_Y,MPU6050_ACC_LAST_Z;
int MPU6050_GYRO_LAST_X,MPU6050_GYRO_LAST_Y,MPU6050_GYRO_LAST_Z;
//陀螺仪加速度计滑动窗口滤波数组
int ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
int GYRO_X_BUF[FILTER_NUM],GYRO_Y_BUF[FILTER_NUM],GYRO_Z_BUF[FILTER_NUM];
//陀螺仪加速度计平均值滤波后的值
int   ACC_AVG_X,ACC_AVG_Y,ACC_AVG_Z;
int   GYRO_AVG_X,GYRO_AVG_Y,GYRO_AVG_Z;
//陀螺仪积分值
float GYRO_I_X,GYRO_I_Y,GYRO_I_Z;



void mpu6050_delay()//长延时
{
    SysCtlDelay(10000);
    SysCtlDelay(10000);
    SysCtlDelay(10000);
    SysCtlDelay(10000);
    SysCtlDelay(10000);
    SysCtlDelay(10000);
    SysCtlDelay(10000);
    SysCtlDelay(10000);
    SysCtlDelay(10000);
}
void mpu6050_init()
{
	SLAVE_ADDRESS=0x68;
	i2c_write(PWR_MGMT_1,0x00);//唤醒mpu6050
    i2c_write(SMPLRT_DIV, 0x00);//设置采样率为500hz，因为后面开了低通滤波，因此输出频率为1khz，1k/(1+1)=500hz
    SysCtlDelay(1000);
    i2c_write(CONFIG, 0x00);//04hz低通滤波
    SysCtlDelay(1000);
    i2c_write(CONFIG, 0x04);//40hz低通滤波
    SysCtlDelay(1000);
    i2c_write(GYRO_CONFIG, 0x08);//陀螺仪量程+-1000度/秒，无自检
    i2c_write(ACCEL_CONFIG, 0x08);//加速度计量程+-8g，无自检
    mpu6050_delay();//长延时，等待6050数据稳定后再取零点
    //MPU6050_ZERO();//mpu6050取零点
}
void MPU6050_ZERO(void)//mpu6050取零点程序
{
	unsigned char i;
	unsigned char num=20;
	long int temp_1=0,temp_2=0,temp_3=0;//,temp_4=0,temp_5=0,temp_6=0;
	//陀螺仪直接校准，加速度计Z轴注意校准重力加速度
	for(i=0;i<num;i++)
	{
		temp_1+=Get16Bit(GYRO_XOUT_H);
		temp_2+=Get16Bit(GYRO_YOUT_H);
		temp_3+=Get16Bit(GYRO_ZOUT_H);
		//temp_4+=Get16Bit(ACCEL_XOUT_H);
		//temp_5+=Get16Bit(ACCEL_YOUT_H);
		//temp_6+=Get16Bit(ACCEL_ZOUT_H)-65536/8;//量程为+-4g
	}
	GYRO_OFFSET_X=(int)temp_1/num;
	GYRO_OFFSET_Y=(int)temp_2/num;
	GYRO_OFFSET_Z=(int)temp_3/num;
	//ACC_OFFSET_X=temp_4/num;
	//ACC_OFFSET_Y=temp_5/num;
	//ACC_OFFSET_Z=temp_6/num;
}
void MPU6050_Dataanl(void)//得到mpu6050数据并减去补偿量
{
  SLAVE_ADDRESS=0x68;
  MPU6050_ACC_LAST_X = Get16Bit(ACCEL_XOUT_H);
  MPU6050_ACC_LAST_Y = Get16Bit(ACCEL_YOUT_H);
  MPU6050_ACC_LAST_Z = Get16Bit(ACCEL_ZOUT_H);

  MPU6050_GYRO_LAST_X = Get16Bit(GYRO_XOUT_H) - GYRO_OFFSET_X;
  MPU6050_GYRO_LAST_Y = Get16Bit(GYRO_YOUT_H) - GYRO_OFFSET_Y;
  MPU6050_GYRO_LAST_Z = Get16Bit(GYRO_ZOUT_H) - GYRO_OFFSET_Z+5;
}
void Prepare_Data(void)//读取MPU6050数据进行平滑滤波，为后续计算准备数据
{
  static unsigned char filter_cnt=0;
  long int temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0;
  unsigned char i;
  MPU6050_Dataanl();//完成传感器数据的读取和计算
  ACC_X_BUF[filter_cnt] = MPU6050_ACC_LAST_X;//更新滑动窗口数组
  ACC_Y_BUF[filter_cnt] = MPU6050_ACC_LAST_Y;
  ACC_Z_BUF[filter_cnt] = MPU6050_ACC_LAST_Z;
  GYRO_X_BUF[filter_cnt] = MPU6050_GYRO_LAST_X;
  GYRO_Y_BUF[filter_cnt] = MPU6050_GYRO_LAST_Y;
  GYRO_Z_BUF[filter_cnt] = MPU6050_GYRO_LAST_Z;
  for(i=0;i<FILTER_NUM;i++)
  {
    temp1 += ACC_X_BUF[i];
    temp2 += ACC_Y_BUF[i];
    temp3 += ACC_Z_BUF[i];
    temp4 += GYRO_X_BUF[i];
    temp5 += GYRO_Y_BUF[i];
    temp6 += GYRO_Z_BUF[i];
  }
  ACC_AVG_X = temp1 / FILTER_NUM;
  ACC_AVG_Y = temp2 / FILTER_NUM;
  ACC_AVG_Z = temp3 / FILTER_NUM;
  GYRO_AVG_X = temp4 / FILTER_NUM;
  GYRO_AVG_Y = temp5 / FILTER_NUM;
  GYRO_AVG_Z = temp6 / FILTER_NUM;
  /*if(check_end==1)
  {
	  if(GYRO_AVG_X<10&&GYRO_AVG_X>-10)
		  GYRO_AVG_X=0;
	  if(GYRO_AVG_Y<10&&GYRO_AVG_Y>-10)
		  GYRO_AVG_Y=0;
	  if(GYRO_AVG_Z<10&&GYRO_AVG_Z>-10)
		  GYRO_AVG_Z=0;
  }*/

  filter_cnt++;
  if(filter_cnt==FILTER_NUM)
  {
	  filter_cnt=0;
  }
  //陀螺仪积分，没有磁力计修正的情况下yaw值由陀螺仪积分得到
  GYRO_I_Z += (MPU6050_GYRO_LAST_Z*Gyro_G*0.0025f);
}
