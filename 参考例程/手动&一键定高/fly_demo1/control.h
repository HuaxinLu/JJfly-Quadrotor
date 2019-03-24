/*********************************************************************************
JJ_fly by TIVA
文件名：control.h
描述：控制程序
作者：卢华歆
时间：2015.11
**********************************************************************************/
#ifndef CONTROL_H_
#define CONTROL_H_
/*
//油门
#define CH_1_MAX 1960
#define CH_1_MID 1476
#define CH_1_MIN 1000
//偏航
#define CH_2_MAX 1956
#define CH_2_MID 1476
#define CH_2_MIN 996
//辅助
#define CH_3_MAX 1956
#define CH_3_MID 1476
#define CH_3_MIN 996
//待定
#define CH_4_MAX 1000
#define CH_4_MID 1000
#define CH_4_MIN 1000
//翻滚
#define CH_5_MAX 1962
#define CH_5_MID 1482
#define CH_5_MIN 1002
//俯仰
#define CH_6_MAX 1978
#define CH_6_MID 1478
#define CH_6_MIN 1018
*/
//油门
#define CH_1_MAX 200
#define CH_1_MID 149
#define CH_1_MIN 101
//偏航
#define CH_2_MAX 200
#define CH_2_MID 149
#define CH_2_MIN 101
//辅助
/*
#define CH_3_STOP 90
#define CH_3_TEST 117
#define CH_3_HAND 151
#define CH_3_HEIGHT 184
*/

#define CH_3_STOP 133
#define CH_3_TEST 172
#define CH_3_HAND 199
#define CH_3_HEIGHT 187

//待定
#define CH_4_MAX 200
#define CH_4_MID 150
#define CH_4_MIN 101
//翻滚
#define CH_5_MAX 200
#define CH_5_MID 149
#define CH_5_MIN 101
//俯仰
#define CH_6_MAX 200
#define CH_6_MID 149
#define CH_6_MIN 101
extern float T;//周期
extern int thr;//油门
extern int motor_1,motor_2,motor_3,motor_4;//四路电机pwm输出
extern float error_angle_x,error_angle_y,error_angle_z;//角度误差
extern float except_angle_x,except_angle_y,except_angle_z;//期望角度
extern float offset_angle_x,offset_angle_y,offset_angle_z;//角度补偿
extern float i_error_angle_x,i_error_angle_y,i_error_angle_z;//角度误差积分值
extern float i_error_angle_x_max,i_error_angle_y_max,i_error_angle_z_max;//积分限幅最大值
extern float i_error_angle_x_min,i_error_angle_y_min,i_error_angle_z_min;//积分限幅最小值
extern float d_error_angle_x,d_error_angle_y,d_error_angle_z;//角度误差微分值
extern float control_1_x,control_1_y,control_1_z;//外环pid输出
extern float old_error_angle_x,old_error_angle_y,old_error_angle_z;//历史数据
extern float kp_angle_x,ki_angle_x,kd_angle_x,kp_angle_y,ki_angle_y,kd_angle_y,kp_angle_z,ki_angle_z,kd_angle_z;//外环pid参数

extern float error_aspeed_x,error_aspeed_y,error_aspeed_z;//角速度误差
extern float except_aspeed_x,except_aspeed_y,except_aspeed_z;//期望角速度
extern float offset_aspeed_x,offset_aspeed_y,offset_aspeed_z;//角速度补偿
extern float i_error_aspeed_x,i_error_aspeed_y,i_error_aspeed_z;//角速度误差积分值
extern float i_error_aspeed_x_max,i_error_aspeed_y_max,i_error_aspeed_z_max;//积分限幅最大值
extern float i_error_aspeed_x_min,i_error_aspeed_y_min,i_error_aspeed_z_min;//积分限幅最小值
extern float d_error_aspeed_x,d_error_aspeed_y,d_error_aspeed_z;//角速度误差微分值
extern float control_2_x,control_2_y,control_2_z;//内环pid输出
extern float old_error_aspeed_x,old_error_aspeed_y,old_error_aspeed_z;//历史数据
extern float kp_aspeed_x,ki_aspeed_x,kd_aspeed_x,kp_aspeed_y,ki_aspeed_y,kd_aspeed_y,kp_aspeed_z,ki_aspeed_z,kd_aspeed_z;//内环pid参数
extern unsigned char ctrl_height;
extern float except_height;
extern float kp_height,ki_height,kd_height,ki_height_ready;
extern float kp_point,kd_point,ki_point;
extern int buchang;
void control();
void control_init();
void control_height();
void control_point();//定点控制
void land();//一键降落
void control_point_1();//定点控制1
#endif /* CONTROL_H_ */
