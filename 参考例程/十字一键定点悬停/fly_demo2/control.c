/*********************************************************************************
JJ_fly by TIVA
文件名：control.c
描述：控制程序
作者：卢华歆
时间：2015.11
**********************************************************************************/
#include "imu_new.h"
#include "mpu6050.h"
#include "math.h"
#include "PWM.h"
#include "control.h"
#include "capture.h"
#include "mymath.h"
#include "US100.h"
#include "flow.h"
float T;//周期
int thr;//油门
int motor_1,motor_2,motor_3,motor_4;//四路电机pwm输出
float error_angle_x,error_angle_y,error_angle_z;//角度误差
float except_angle_x,except_angle_y,except_angle_z;//期望角度
float offset_angle_x,offset_angle_y,offset_angle_z;//角度补偿
float i_error_angle_x,i_error_angle_y,i_error_angle_z;//角度误差积分值
float i_error_angle_x_max,i_error_angle_y_max,i_error_angle_z_max;//积分限幅最大值
float i_error_angle_x_min,i_error_angle_y_min,i_error_angle_z_min;//积分限幅最小值
float d_error_angle_x,d_error_angle_y,d_error_angle_z;//角度误差微分值
float control_1_x,control_1_y,control_1_z;//外环pid输出
float old_error_angle_x,old_error_angle_y,old_error_angle_z;//历史数据
float kp_angle_x,ki_angle_x,kd_angle_x,kp_angle_y,ki_angle_y,kd_angle_y,kp_angle_z,ki_angle_z,kd_angle_z;//外环pid参数

float error_aspeed_x,error_aspeed_y,error_aspeed_z;//角速度误差
float except_aspeed_x,except_aspeed_y,except_aspeed_z;//期望角速度
float offset_aspeed_x,offset_aspeed_y,offset_aspeed_z;//角速度补偿
float i_error_aspeed_x,i_error_aspeed_y,i_error_aspeed_z;//角速度误差积分值
float i_error_aspeed_x_max,i_error_aspeed_y_max,i_error_aspeed_z_max;//积分限幅最大值
float i_error_aspeed_x_min,i_error_aspeed_y_min,i_error_aspeed_z_min;//积分限幅最小值
float d_error_aspeed_x,d_error_aspeed_y,d_error_aspeed_z;//角速度误差微分值
float control_2_x,control_2_y,control_2_z;//内环pid输出
float old_error_aspeed_x,old_error_aspeed_y,old_error_aspeed_z;//历史数据
float kp_aspeed_x,ki_aspeed_x,kd_aspeed_x,kp_aspeed_y,ki_aspeed_y,kd_aspeed_y,kp_aspeed_z,ki_aspeed_z,kd_aspeed_z;//内环pid参数
int THR_keep;

//高度控制
unsigned char mode_height;
unsigned char mode_land;
float error_height , except_height;
float kp_height,ki_height,kd_height,ki_height_ready;
float ctrl_height_out,error_height,i_error_height,d_error_height,old_error_height;
unsigned char error_flag;
unsigned char ctrl_height;
unsigned char fly_low;
float fly_thr;//基础油门
//定点
float except_x_angle;
float except_y_angle;
float kp_point,kd_point,ki_point,ki_dis;
float d_error_vx,d_error_vy,error_vx,error_vy,i_error_vx,i_error_vy;
float old_error_vx,old_error_vy;
unsigned char point_flag;
float i_error_X,i_error_Y;
float error_X,error_Y,d_error_X,d_error_Y,old_error_X,old_error_Y;
int buchang;
void control_init()
{
	//设定pid参数
	kp_angle_x=580;
	ki_angle_x=5;
	kd_angle_x=0;
	kp_angle_y=580;
	ki_angle_y=5;
	kd_angle_y=0;
	kp_angle_z=700;
	ki_angle_z=0;
	kd_angle_z=0;

	kp_aspeed_x=42;
	ki_aspeed_x=0;
	kd_aspeed_x=1500;
	kp_aspeed_y=42;
	ki_aspeed_y=0;
	kd_aspeed_y=1500;
	kp_aspeed_z=300;
	ki_aspeed_z=0;
	kd_aspeed_z=400;
	//设定积分限幅
	i_error_angle_x_max=1200;
	i_error_angle_y_max=1200;
	i_error_angle_z_max=1200;

	i_error_angle_x_min=-1200;
	i_error_angle_y_min=-1200;
	i_error_angle_z_min=-1200;

	i_error_aspeed_x_max=400;
	i_error_aspeed_y_max=400;
	i_error_aspeed_z_max=2000;

	i_error_aspeed_x_min=-400;
	i_error_aspeed_y_min=-400;
	i_error_aspeed_z_min=-2000;
	//设定周期
	except_height=9.0f;
	kp_height=120;
	kd_height=1000;
	ki_height=11;
	ki_height_ready=38;
	T=5;//pwm刷新为5ms
	//定点参数
	kp_point=150;
	ki_point=30;
	kd_point=750;
	buchang=15;
}
void control_1()//x roll,y pitch,z yaw
{
	//得到角度误差
	error_angle_x = except_angle_x + offset_angle_x - Q_ANGLE_X;
	error_angle_y = except_angle_y + offset_angle_y - Q_ANGLE_Y;
	error_angle_z = except_angle_z + offset_angle_z - Q_ANGLE_Z;
	if(error_angle_x<=0.1f&&error_angle_x>=-0.1f)
	{
		error_angle_x=0;
	}
	if(error_angle_y<=0.1f&&error_angle_y>=-0.1f)
	{
		error_angle_y=0;
	}
	if(error_angle_z<=0.2f&&error_angle_z>=-0.2f)
	{
		error_angle_z=0;
	}
	if(error_angle_z>50.0f||error_angle_z<-50.0f)
	{
		error_angle_z=0;
	}
	//角度误差积分
	i_error_angle_x = (i_error_angle_x + error_angle_x);
	i_error_angle_y = (i_error_angle_y + error_angle_y);
	i_error_angle_z = (i_error_angle_z + error_angle_z);
	//角度误差积分限幅
	if(i_error_angle_x > i_error_angle_x_max) i_error_angle_x = i_error_angle_x_max;
	if(i_error_angle_x < i_error_angle_x_min) i_error_angle_x = i_error_angle_x_min;
	if(i_error_angle_y > i_error_angle_y_max) i_error_angle_y = i_error_angle_y_max;
	if(i_error_angle_y < i_error_angle_y_min) i_error_angle_y = i_error_angle_y_min;
	if(i_error_angle_z > i_error_angle_z_max) i_error_angle_z = i_error_angle_z_max;
	if(i_error_angle_z < i_error_angle_z_min) i_error_angle_z = i_error_angle_z_min;
	//角度误差微分
	d_error_angle_x = (error_angle_x - old_error_angle_x) / T;
	d_error_angle_y = (error_angle_y - old_error_angle_y) / T;
	d_error_angle_z = (error_angle_z - old_error_angle_z) / T;
	//外环pid输出
	control_1_x = kp_angle_x * error_angle_x + ki_angle_x * i_error_angle_x / 100 + kd_angle_x * d_error_angle_x;
	control_1_y = kp_angle_y * error_angle_y + ki_angle_y * i_error_angle_y / 100+ kd_angle_y * d_error_angle_y;
	control_1_z = kp_angle_z * error_angle_z + ki_angle_z * i_error_angle_z / 100+ kd_angle_z * d_error_angle_z;
	//保存历史数据
	old_error_angle_x = error_angle_x;
	old_error_angle_y = error_angle_y;
	old_error_angle_z = error_angle_z;
	if(CH_2<CH_2_MID-2||CH_2>CH_2_MID+2)
	{
		control_1_z=0;//不打舵锁定航向
		except_angle_z=Q_ANGLE_Z;
	}
	else
	{
		error_Y=0;
	}
}
void control_2()//x roll,y pitch,z yaw
{
	//得到角速度误差
	error_aspeed_x = except_aspeed_x + offset_aspeed_x - MPU6050_GYRO_LAST_X*Gyro_G;
	error_aspeed_y = except_aspeed_y + offset_aspeed_y - MPU6050_GYRO_LAST_Y*Gyro_G;
	error_aspeed_z = except_aspeed_z + offset_aspeed_z - MPU6050_GYRO_LAST_Z/100;
	//角速度误差积分
	i_error_aspeed_x = (i_error_aspeed_x + error_aspeed_x)/10000;
	i_error_aspeed_y = (i_error_aspeed_y + error_aspeed_y)/10000;
	i_error_aspeed_z = (i_error_aspeed_z + error_aspeed_z)/10000;

	//角速度误差积分限幅
	if(i_error_aspeed_x > i_error_aspeed_x_max) i_error_aspeed_x = i_error_aspeed_x_max;
	if(i_error_aspeed_x < i_error_aspeed_x_min) i_error_aspeed_x = i_error_aspeed_x_min;
	if(i_error_aspeed_y > i_error_aspeed_y_max) i_error_aspeed_y = i_error_aspeed_y_max;
	if(i_error_aspeed_y < i_error_aspeed_y_min) i_error_aspeed_y = i_error_aspeed_y_min;
	if(i_error_aspeed_z > i_error_aspeed_z_max) i_error_aspeed_z = i_error_aspeed_z_max;
	if(i_error_aspeed_z < i_error_aspeed_z_min) i_error_aspeed_z = i_error_aspeed_z_min;

	//角速度误差微分
	d_error_aspeed_x = (error_aspeed_x - old_error_aspeed_x) / T;
	d_error_aspeed_y = (error_aspeed_y - old_error_aspeed_y) / T;
	d_error_aspeed_z = (error_aspeed_z - old_error_aspeed_z) / T;
	//内环pid输出
	control_2_x = kp_aspeed_x * error_aspeed_x + ki_aspeed_x * i_error_aspeed_x + kd_aspeed_x * d_error_aspeed_x;
	control_2_y = kp_aspeed_y * error_aspeed_y + ki_aspeed_y * i_error_aspeed_y + kd_aspeed_y * d_error_aspeed_y;
	control_2_z = kp_aspeed_z * error_aspeed_z + ki_aspeed_z * i_error_aspeed_z + kd_aspeed_z * d_error_aspeed_z;
	//保存历史数据
	old_error_aspeed_x = error_aspeed_x;
	old_error_aspeed_y = error_aspeed_y;
	old_error_aspeed_z = error_aspeed_z;
}
//一键起飞通过改变高度误差积分常数I实现
//当离地之前，I取的比较大（ki_height_ready），当判断起飞之后，I取的比较小（ki_height）
void control_height()
{
		//高度环
		if(mode_land==0)
		{
			except_height=(CH_1-CH_1_MIN)/5.0f+0.4f;
		}
		//得到高度误差
		error_height = except_height - US100_Alt;

		//高度误差微分
		d_error_height = (error_height - old_error_height);
		if(US100_Alt<2.5f&&fly_low==0)
		{
			//高度误差积分
			i_error_height = (i_error_height + ki_height_ready*error_height);
			fly_thr=0;
		}
		//起飞高度判断为20cm
		if(US100_Alt>2.5f&&fly_low==0)
		{
			fly_low=1;
			fly_thr=i_error_height;
			i_error_height=0;
		}
		if(fly_low==1)
		{
			i_error_height = (i_error_height + ki_height*error_height);
		}
		//高度误差积分限幅
		if(i_error_height > 16000.0f) i_error_aspeed_x = 16000.0f;
		if(i_error_height < -16000.0f) i_error_aspeed_x = -16000.0f;
		//高度环
		if(mode_land==0)
		{
			ctrl_height_out = fly_thr+kp_height * error_height + i_error_height + kd_height * d_error_height;
		}
		else
		{
			ctrl_height_out = fly_thr+kp_height * error_height + i_error_height + 200.0f * d_error_height;
		}
		//保存历史数据
		old_error_height = error_height;
}
void land()//一键降落
{
	if(mode_land==1)
	{
		//if(except_height>1.0f)
		{
			except_height=except_height-0.8f;
		}
		/*if(US100_Alt<2.5f)
		{
			fly_thr=fly_thr-100;
		}*/

	}
}
//对于ADNS3080的控制定点的程序，由于3080积分误差很大，不宜采用积分控制
void control_point()//定点控制
{
	error_vx=vx-127;
	error_vy=vy-127;

	if(error_vx>-2&&error_vx<2)
		error_vx=0;
	if(error_vy>-2&&error_vy<2)
		error_vy=0;
	d_error_vx = (error_vx - old_error_vx);
	d_error_vy = (error_vy - old_error_vy);
    if(point_flag==1)
    {
		i_error_vx=i_error_vx+error_vx;
		i_error_vy=i_error_vy+error_vy;
    }
	if(i_error_vx>50) i_error_vx=50;
	if(i_error_vx<-50) i_error_vx=-50;
	if(i_error_vy>50) i_error_vy=50;
	if(i_error_vy<-50) i_error_vy=-50;
	except_x_angle=kp_point*0.01f*error_vx+kd_point*0.01f*d_error_vx+ki_point*0.001f*i_error_vx;
	except_y_angle=kp_point*0.01f*error_vy+kd_point*0.01f*d_error_vy+ki_point*0.01f*i_error_vy;

	if(except_x_angle>3.0f) except_x_angle=3.0f;
	if(except_x_angle<-3.0f) except_x_angle=-3.0f;
	if(except_y_angle>3.0f) except_y_angle=3.0f;
	if(except_y_angle<-3.0f) except_y_angle=-3.0f;

	old_error_vx=error_vx;
	old_error_vy=error_vy;
}
void control_point_1()//定点控制1
{
	error_X=X-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
	error_Y=Y-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
	if(US100_Alt<0.6f)
	{
		error_X=0;
		error_Y=0;
	}
	if(Y==0)
	{
		error_Y=0;
	}
	if(X==0)
	{
		error_X=0;
	}
	/*if(error_vx>-2&&error_vx<2)
		error_vx=0;
	if(error_vy>-2&&error_vy<2)
		error_vy=0;*/
	d_error_X = (error_X - old_error_X);
	d_error_Y = (error_Y - old_error_Y);
	if(US100_Alt>2.0f)
	{
		i_error_X+=error_X;
		i_error_Y+=error_Y;
	}
	except_x_angle=kp_point*0.001f*error_X+kd_point*0.001f*d_error_X+ki_point*0.0001f*i_error_X;
	except_y_angle=kp_point*0.001f*error_Y+kd_point*0.001f*d_error_Y+ki_point*0.0001f*i_error_Y;

	if(except_x_angle>2.0f) except_x_angle=2.0f;
	if(except_x_angle<-2.0f) except_x_angle=-2.0f;
	if(except_y_angle>2.0f) except_y_angle=2.0f;
	if(except_y_angle<-2.0f) except_y_angle=-2.0f;

	old_error_X=error_X;
	old_error_Y=error_Y;
}
void control()
{
	except_angle_x=(CH_5-CH_5_MID)/1.0f*(-1)-except_y_angle;
	except_angle_y=(CH_6-CH_6_MID)/1.0f-except_x_angle;
	except_aspeed_z=(CH_2-CH_2_MID)*4.0f*(-1);//z轴控制角速度
	control_1();//外环
	control_2();//内环
	if(CH_1<CH_1_MIN||CH_1>CH_1_MAX)//油门异常值排除
	{
		CH_1=CH_1_MIN;
	}

	if(CH_3<CH_3_HEIGHT+5&&CH_3>CH_3_HEIGHT-5)//解锁，容错10，辅助开关打到最高解锁，同时z轴开始积分
	{
		/////定高模式
		mode_height=1;//开启定高模式
		motor_1 = 20000 + ctrl_height_out - control_1_x - control_1_y - control_1_z + control_2_x - control_2_y + control_2_z;
		motor_2 = 20000 + ctrl_height_out + control_1_x - control_1_y + control_1_z - control_2_x - control_2_y - control_2_z;
		motor_3 = 20000 + ctrl_height_out + control_1_x + control_1_y - control_1_z - control_2_x + control_2_y + control_2_z;
		motor_4 = 20000 + ctrl_height_out - control_1_x + control_1_y + control_1_z + control_2_x + control_2_y - control_2_z;
	}
	else if(CH_3<CH_3_HAND+5&&CH_3>CH_3_HAND-5)//解锁，容错10，辅助开关打到最高解锁，同时z轴开始积分
	{
		/////手动模式
		if(mode_height==0)
		{
			motor_1 = 20000 +(CH_1 - CH_1_MIN) * 200 - control_1_x - control_1_y - control_1_z + control_2_x - control_2_y + control_2_z;
			motor_2 = 20000 +(CH_1 - CH_1_MIN) * 200 + control_1_x - control_1_y + control_1_z - control_2_x - control_2_y - control_2_z;
			motor_3 = 20000 +(CH_1 - CH_1_MIN) * 200 + control_1_x + control_1_y - control_1_z - control_2_x + control_2_y + control_2_z;
			motor_4 = 20000 +(CH_1 - CH_1_MIN) * 200 - control_1_x + control_1_y + control_1_z + control_2_x + control_2_y - control_2_z;

		}
		else if(mode_height==1)//定高的时候打开这个开关代表开启降落模式
		{
			mode_land=1;//开启降落模式
			motor_1 = 20000 + ctrl_height_out - control_1_x - control_1_y - control_1_z + control_2_x - control_2_y + control_2_z;
			motor_2 = 20000 + ctrl_height_out + control_1_x - control_1_y + control_1_z - control_2_x - control_2_y - control_2_z;
			motor_3 = 20000 + ctrl_height_out + control_1_x + control_1_y - control_1_z - control_2_x + control_2_y + control_2_z;
			motor_4 = 20000 + ctrl_height_out - control_1_x + control_1_y + control_1_z + control_2_x + control_2_y - control_2_z;

		}
		//
	}
	else //未解锁，限制pwm输出、z轴角速度
	{
		motor_1 = 20000;
		motor_2 = 20000;
		motor_3 = 20000;
		motor_4 = 20000;
		GYRO_I_Z=0;//陀螺仪积分清零
		error_flag=0;
		i_error_aspeed_z=0;
		Az=0;
		i_error_height=0;
		fly_thr=0;
		fly_low=0;
		except_angle_z=0;
		mode_land=0;
		mode_height=0;
		i_error_vx=0;
		i_error_vy=0;
		point_flag=0;
		//offset_angle_x = Q_ANGLE_X;
		//offset_angle_y = Q_ANGLE_Y;
		X=0;
		Y=0;
	}
	if(CH_1<CH_1_MIN+2)//油门打到最低禁止输出
	{
		motor_1 = 20000;
		motor_2 = 20000;
		motor_3 = 20000;
		motor_4 = 20000;
		i_error_aspeed_x=0;
		i_error_aspeed_y=0;
		i_error_angle_x=0;
		i_error_angle_y=0;
		i_error_X=0;
		i_error_Y=0;
	}
	if(US100_Alt<1.8f&&mode_land==1)
	{
		error_flag=1;//关闭电机
		mode_land=0;
		mode_height=0;
	}
	if(Q_ANGLE_X>50 || Q_ANGLE_X<-50 || Q_ANGLE_Y>50 || Q_ANGLE_Y<-50 )
	{
		error_flag=1;
	}
	if(error_flag==1)//角度出现异常值
	{
		motor_1 = 20000;
		motor_2 = 20000;
		motor_3 = 20000;
		motor_4 = 20000;
	}

	//pwm波限幅，20000对应1ms，40000对应2ms
	if(motor_1<20000) motor_1=20000;
	if(motor_1>40000) motor_1=40000;
	if(motor_2<20000) motor_2=20000;
	if(motor_2>40000) motor_2=40000;
	if(motor_3<20000) motor_3=20000;
	if(motor_3>40000) motor_3=40000;
	if(motor_4<20000) motor_4=20000;
	if(motor_4>40000) motor_4=40000;
	//刷新pwm值
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, motor_2);//m2
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, motor_1);//m1
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, motor_4);//m4
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, motor_3);//m3
}
