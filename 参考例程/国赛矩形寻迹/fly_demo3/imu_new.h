/*********************************************************************************
JJ_fly by TIVA
文件名：imu_new.h
描述：姿态解算最终版
作者：卢华歆
时间：2015.11
**********************************************************************************/

#ifndef IMU_NEW_H_
#define IMU_NEW_H_
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	鏃犵鍙?浣嶆暣鍨嬪彉閲? */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		鏈夌鍙?浣嶆暣鍨嬪彉閲? */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	鏃犵鍙?6浣嶆暣鍨嬪彉閲?*/
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		鏈夌鍙?6浣嶆暣鍨嬪彉閲?*/
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	鏃犵鍙?2浣嶆暣鍨嬪彉閲?*/
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		鏈夌鍙?2浣嶆暣鍨嬪彉閲?*/
typedef float          fp32;                    /* single precision floating point variable (32bits) 鍗曠簿搴︽诞鐐规暟锛?2浣嶉暱搴︼級 */
typedef double         fp64;
#define RtA 		57.324841f		//  180/3.1415
#define AtR    	0.0174533f		//  1/RtA
#define Acc_G 	0.0011963f		//  1/32768/4/9.8
//#define Gyro_G 	0.03051756f/2f	//  1/32768/1000/2
//#define Gyro_Gr	0.0005326f/2f   //  1/32768/1000/57.3

extern float Az;
extern float Q_ANGLE_X,Q_ANGLE_Y,Q_ANGLE_Z;      //四元数计算出的角度
extern float Q_ANGLE_Xr,Q_ANGLE_Yr;
extern float Gx,Gy,Gz;
void Get_Attitude();
#endif /* IMU_NEW_H_ */
