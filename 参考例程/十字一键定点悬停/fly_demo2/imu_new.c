/*********************************************************************************
JJ_fly by TIVA
文件名：imu_new.c
描述：姿态解算最终版
作者：卢华歆
时间：2015.11
**********************************************************************************/
#include "imu_new.h"
#include "mpu6050.h"
#include "math.h"
#include "mymath.h"
#include "init.h"
float Q_ANGLE_X,Q_ANGLE_Y,Q_ANGLE_Z;      //閸ユ稑鍘撻弫鎷岊吀缁犳鍤惃鍕瀾鎼达拷
float Q_ANGLE_Xr,Q_ANGLE_Yr;
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;
	i  = 0x5f3759df - ( i >> 1 );
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration 鎷㈢瘬纰岃劮鑴楹撹劍鑴滄嫝闇茶劦纰岀湁楹撶叅鎷㈡紡
	return y;
}
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x;
}
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result;
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result;
}
float VariableParameter(float error)
{
	float  result = 0;

	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.25 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}

/*************************************/

#define Kp 5.0f                        // 用于加速度计修正陀螺仪的偏差
#define Ki 0.00015f                     // 用于修正陀螺仪的误差积分
#define halfT 0.0025f                 // 姿态解算周期的一半 单位S
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 四元数
float exInt = 0, eyInt = 0, ezInt = 0;    // 积分误差
#define REF_ERR_LPF_HZ  1				//(Hz)
float ex_lpf, ey_lpf;
float ex_tmp, ey_tmp;
float Gx,Gy,Gz;
float Az;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float temp;
	//float ref_err_lpf_hz;
	float norm;
    //int16_t Xr,Yr;
	float vx, vy, vz;// wx, wy, wz;
	float ex, ey, ez;
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	//  float q0q3 = q0*q3;//
	float q1q1 = q1*q1;
	//  float q1q2 = q1*q2;//
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	norm = my_sqrt(ax*ax + ay*ay + az*az);
	//这段很重要，对每一次的加速度模判断并和G做比较，决定是否采用加速度计数据
	if(norm>G-30&&norm<G+30)
	{
		ax = ax /norm;
		ay = ay / norm;
		az = az / norm;
		// 估计重力流迁
		vx = 2*(q1q3 - q0q2);
		vy = 2*(q0q1 + q2q3);
		vz = q0q0 - q1q1 - q2q2 + q3q3 ;
		// 得到误差
		ex = (ay*vz - az*vy) ;
		ey = (az*vx - ax*vz) ;
		ez = (ax*vy - ay*vx) ;
		/* 一阶低通滤波*/
		//ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *halfT);
		//ex_lpf += ref_err_lpf_hz *( ex_tmp  - ex_lpf );
		//ey_lpf += ref_err_lpf_hz *( ey_tmp  - ey_lpf );
        //ex=ex_lpf;
        //ey=ex_lpf;
	}
	else
	{
		ex = 0 ;
		ey = 0 ;
		ez = 0 ;
	}
	exInt = exInt +  ex * Ki;
	eyInt = eyInt +  ey * Ki;
	ezInt = ezInt +  ez * Ki;
	// 补偿陀螺仪的漂移
	gx = gx + Kp *   ex + exInt;
	gy = gy + Kp *   ey + eyInt;
	gz = gz + Kp *   ez + ezInt;

	// 获得四元数
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	// 提取重力的模
	norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	//滤除有害加速度
	if(check_end==1)
	{
		Gx = 2*(q1q3 - q0q2);
		Gy = 2*(q0q1 + q2q3);
		Gz = 1-2*(q1q1 +  q2q2);
		temp= (Gz *(ACC_AVG_Z- ACC_OFFSET_Z) + ax *Gx + ay *Gy)/100;//
		if((temp>40.0f||temp<-40.0f)&&(temp>-250.0f||temp<250.0f))
		{
			Az=Az+temp;
		}
	}
	Q_ANGLE_Xr = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1); // roll
	Q_ANGLE_Yr = asin(-2*q1q3 + 2*q0q2); // pitch
	//Q_ANGLE_Z = atan2((double)Yr,(double)Xr) * RtA; // yaw
	Q_ANGLE_X = To_180_degrees(Q_ANGLE_Xr *(-1)*RtA);
	Q_ANGLE_Y = To_180_degrees(Q_ANGLE_Yr*RtA);
	Q_ANGLE_Z = (-1)*To_180_degrees(GYRO_I_Z/2);
}
void Get_Attitude(void)//婵寧锟界憴锝囩暬
{
	//ACC_AVG_X = KalmanFilter_x(ACC_AVG_X,KALMAN_Q,KALMAN_R);  // ACC X鏉炴潙宕辩亸鏃�禆濠娿倖灏�
	//ACC_AVG_Y = KalmanFilter_y(ACC_AVG_Y,KALMAN_Q,KALMAN_R);  // ACC Y鏉炴潙宕辩亸鏃�禆濠娿倖灏�
	//ACC_AVG_Z = KalmanFilter_z(ACC_AVG_Z,KALMAN_Q,KALMAN_R);  // ACC Z鏉炴潙宕辩亸鏃�禆濠娿倖灏�
  //IMUupdate(GYRO_AVG_X*Gyro_Gr, GYRO_AVG_Y*Gyro_Gr,GYRO_AVG_Z*Gyro_Gr,ACC_AVG_X,ACC_AVG_Y,ACC_AVG_Z);	//*0.0174鏉烆剚鍨氬褍瀹�
  IMUupdate(MPU6050_GYRO_LAST_X*Gyro_Gr, MPU6050_GYRO_LAST_Y*Gyro_Gr,MPU6050_GYRO_LAST_Z*Gyro_Gr,ACC_AVG_X,ACC_AVG_Y,ACC_AVG_Z);	//*0.0174鏉烆剚鍨氬褍瀹�

}
