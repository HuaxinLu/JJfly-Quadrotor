/*********************************************************************************
JJ_fly by TIVA
文件名：mpu6050.h
描述：mpu6050数据读取与处理，参考匿名旧版程序
作者：卢华歆
时间：2015.10
**********************************************************************************/
#ifndef MPU6050_H_
#define MPU6050_H_

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

#define SMPLRT_DIV	0x19	//陀螺仪采样率
#define	 CONFIG		0x1A	//低通滤波频率
#define	 GYRO_CONFIG	0x1B	//陀螺仪自检及测量范围
#define	 ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率
#define	 ACCEL_XOUT_H	0x3B    //加速度
#define	 ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define	 ACCEL_ZOUT_L	0x40
#define	 TEMP_OUT_H	    0x41
#define	 TEMP_OUT_L	    0x42
#define	 GYRO_XOUT_H	0x43    //角速度
#define	 GYRO_XOUT_L	0x44
#define	 GYRO_YOUT_H	0x45
#define	 GYRO_YOUT_L	0x46
#define	 GYRO_ZOUT_H	0x47
#define	 GYRO_ZOUT_L	0x48
#define	 PWR_MGMT_1	0x6B	//电源管理
#define WHO_AM_I 0x75

#define RtA 		57.324841f		//  180/3.1415  角度制 转化为弧度制
#define AtR    	0.0174533f		//  1/RtA             RtA倒数
#define Acc_G 	0.0011963f		//  1/32768/4/9.8     加速度量程为4G
//#define Gyro_G 	0.0610351/2				//角速度变成度   此参数对应陀螺1000度每秒
//#define Gyro_Gr	0.0010653/2				//角速度变成弧度	此参数对应陀螺1000度每秒
#define Gyro_G 	0.01525878f	//  1/32768/1000      陀螺仪量程为 +—1000
#define Gyro_Gr	0.0002663f   //  1/32768/1000/57.3
#define FILTER_NUM 10//滑动滤波次数

void mpu6050_init();
void MPU6050_ZERO(void);
void Prepare_Data(void);
void MPU6050_Dataanl(void);

//陀螺仪加速度计的补偿量
extern int ACC_OFFSET_X,ACC_OFFSET_Y,ACC_OFFSET_Z;
extern int GYRO_OFFSET_X,GYRO_OFFSET_Y,GYRO_OFFSET_Z;
//陀螺仪加速度计实时值，带修正
extern int MPU6050_ACC_LAST_X,MPU6050_ACC_LAST_Y,MPU6050_ACC_LAST_Z;
extern int MPU6050_GYRO_LAST_X,MPU6050_GYRO_LAST_Y,MPU6050_GYRO_LAST_Z;
//陀螺仪加速度计滑动窗口滤波数组
extern int ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
extern int GYRO_X_BUF[FILTER_NUM],GYRO_Y_BUF[FILTER_NUM],GYRO_Z_BUF[FILTER_NUM];
//陀螺仪加速度计平均值滤波后的值
extern int   ACC_AVG_X,ACC_AVG_Y,ACC_AVG_Z;
extern int   GYRO_AVG_X,GYRO_AVG_Y,GYRO_AVG_Z;
//陀螺仪积分值
extern float GYRO_I_X,GYRO_I_Y,GYRO_I_Z;

#endif /* MPU6050_H_ */
