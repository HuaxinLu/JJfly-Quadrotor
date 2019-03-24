/*********************************************************************************
JJ_fly by TIVA
文件名：mymath.h
描述：匿名的快速计算库
作者：卢华歆
时间：2015.10
**********************************************************************************/
#ifndef __MYMATH_H__
#define __MYMATH_H__

#define REAL   float
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII   3.14159f
#define MY_PPPIII_HALF   1.570796f
#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);
float my_pow(float a);
float my_sqrt(float number);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float x,float zoom);
float To_180_degrees(float x);
float Moving_Median(unsigned char item,unsigned char width_num,float in);
#endif

