/*********************************************************************************
JJ_fly by TIVA
鏂囦欢鍚嶏細control.c
鎻忚堪锛氭帶鍒剁▼搴�
浣滆�锛氬崲鍗庢瓎
鏃堕棿锛�015.11
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
#include "timers.h"
float T;//鍛ㄦ湡
int thr;//娌归棬
int motor_1,motor_2,motor_3,motor_4;//鍥涜矾鐢垫満pwm杈撳嚭
float error_angle_x,error_angle_y,error_angle_z;//瑙掑害璇樊
float except_angle_x,except_angle_y,except_angle_z;//鏈熸湜瑙掑害
float offset_angle_x,offset_angle_y,offset_angle_z;//瑙掑害琛ュ伩
float i_error_angle_x,i_error_angle_y,i_error_angle_z;//瑙掑害璇樊绉垎鍊�
float i_error_angle_x_max,i_error_angle_y_max,i_error_angle_z_max;//绉垎闄愬箙鏈�ぇ鍊�
float i_error_angle_x_min,i_error_angle_y_min,i_error_angle_z_min;//绉垎闄愬箙鏈�皬鍊�
float d_error_angle_x,d_error_angle_y,d_error_angle_z;//瑙掑害璇樊寰垎鍊�
float control_1_x,control_1_y,control_1_z;//澶栫幆pid杈撳嚭
float old_error_angle_x,old_error_angle_y,old_error_angle_z;//鍘嗗彶鏁版嵁
float kp_angle_x,ki_angle_x,kd_angle_x,kp_angle_y,ki_angle_y,kd_angle_y,kp_angle_z,ki_angle_z,kd_angle_z;//澶栫幆pid鍙傛暟

float error_aspeed_x,error_aspeed_y,error_aspeed_z;//瑙掗�搴﹁宸�
float except_aspeed_x,except_aspeed_y,except_aspeed_z;//鏈熸湜瑙掗�搴�
float offset_aspeed_x,offset_aspeed_y,offset_aspeed_z;//瑙掗�搴﹁ˉ鍋�
float i_error_aspeed_x,i_error_aspeed_y,i_error_aspeed_z;//瑙掗�搴﹁宸Н鍒嗗�
float i_error_aspeed_x_max,i_error_aspeed_y_max,i_error_aspeed_z_max;//绉垎闄愬箙鏈�ぇ鍊�
float i_error_aspeed_x_min,i_error_aspeed_y_min,i_error_aspeed_z_min;//绉垎闄愬箙鏈�皬鍊�
float d_error_aspeed_x,d_error_aspeed_y,d_error_aspeed_z;//瑙掗�搴﹁宸井鍒嗗�
float control_2_x,control_2_y,control_2_z;//鍐呯幆pid杈撳嚭
float old_error_aspeed_x,old_error_aspeed_y,old_error_aspeed_z;//鍘嗗彶鏁版嵁
float kp_aspeed_x,ki_aspeed_x,kd_aspeed_x,kp_aspeed_y,ki_aspeed_y,kd_aspeed_y,kp_aspeed_z,ki_aspeed_z,kd_aspeed_z;//鍐呯幆pid鍙傛暟
int THR_keep;

//楂樺害鎺у埗
unsigned char mode_height;
unsigned char mode_land;
float error_height , except_height;
float kp_height,ki_height,kd_height,ki_height_ready;
float ctrl_height_out,error_height,i_error_height,d_error_height,old_error_height;
unsigned char error_flag;
unsigned char ctrl_height;
unsigned char fly_low;
float fly_thr;//鍩虹娌归棬
//瀹氱偣
float except_x_angle;
float except_y_angle;
float kp_point,kd_point,ki_point,ki_dis;
float d_error_vx,d_error_vy,error_vx,error_vy,i_error_vx,i_error_vy;
float old_error_vx,old_error_vy;
unsigned char point_flag;
float i_error_X,i_error_Y;
float error_X,error_Y,d_error_X,d_error_Y,old_error_X,old_error_Y;
int buchang;
int game_speed;
float game_angle_y,game_angle_x;

int shache,shache_flag_2,shache_flag_3,shache_flag_4,shache_flag_5,shache_flag_6;
unsigned char shache_flag;
void control_init()
{
	//璁惧畾pid鍙傛暟
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
	//璁惧畾绉垎闄愬箙
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
	shache=57;
	game_speed=70;
}
void control_1()//x roll,y pitch,z yaw
{
	//寰楀埌瑙掑害璇樊
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
	//瑙掑害璇樊绉垎
	i_error_angle_x = (i_error_angle_x + error_angle_x);
	i_error_angle_y = (i_error_angle_y + error_angle_y);
	i_error_angle_z = (i_error_angle_z + error_angle_z);
	//瑙掑害璇樊绉垎闄愬箙
	if(i_error_angle_x > i_error_angle_x_max) i_error_angle_x = i_error_angle_x_max;
	if(i_error_angle_x < i_error_angle_x_min) i_error_angle_x = i_error_angle_x_min;
	if(i_error_angle_y > i_error_angle_y_max) i_error_angle_y = i_error_angle_y_max;
	if(i_error_angle_y < i_error_angle_y_min) i_error_angle_y = i_error_angle_y_min;
	if(i_error_angle_z > i_error_angle_z_max) i_error_angle_z = i_error_angle_z_max;
	if(i_error_angle_z < i_error_angle_z_min) i_error_angle_z = i_error_angle_z_min;
	//瑙掑害璇樊寰垎
	d_error_angle_x = (error_angle_x - old_error_angle_x) / T;
	d_error_angle_y = (error_angle_y - old_error_angle_y) / T;
	d_error_angle_z = (error_angle_z - old_error_angle_z) / T;
	//澶栫幆pid杈撳嚭
	control_1_x = kp_angle_x * error_angle_x + ki_angle_x * i_error_angle_x / 100 + kd_angle_x * d_error_angle_x;
	control_1_y = kp_angle_y * error_angle_y + ki_angle_y * i_error_angle_y / 100+ kd_angle_y * d_error_angle_y;
	control_1_z = kp_angle_z * error_angle_z + ki_angle_z * i_error_angle_z / 100+ kd_angle_z * d_error_angle_z;
	//淇濆瓨鍘嗗彶鏁版嵁
	old_error_angle_x = error_angle_x;
	old_error_angle_y = error_angle_y;
	old_error_angle_z = error_angle_z;
	if(CH_2<CH_2_MID-2||CH_2>CH_2_MID+2)
	{
		control_1_z=0;//涓嶆墦鑸甸攣瀹氳埅鍚�
		except_angle_z=Q_ANGLE_Z;
	}
	else
	{
		error_Y=0;
	}
}
void control_2()//x roll,y pitch,z yaw
{
	//寰楀埌瑙掗�搴﹁宸�
	error_aspeed_x = except_aspeed_x + offset_aspeed_x - MPU6050_GYRO_LAST_X*Gyro_G;
	error_aspeed_y = except_aspeed_y + offset_aspeed_y - MPU6050_GYRO_LAST_Y*Gyro_G;
	error_aspeed_z = except_aspeed_z + offset_aspeed_z - MPU6050_GYRO_LAST_Z/100;
	//瑙掗�搴﹁宸Н鍒�
	i_error_aspeed_x = (i_error_aspeed_x + error_aspeed_x)/10000;
	i_error_aspeed_y = (i_error_aspeed_y + error_aspeed_y)/10000;
	i_error_aspeed_z = (i_error_aspeed_z + error_aspeed_z)/10000;

	//瑙掗�搴﹁宸Н鍒嗛檺骞�
	if(i_error_aspeed_x > i_error_aspeed_x_max) i_error_aspeed_x = i_error_aspeed_x_max;
	if(i_error_aspeed_x < i_error_aspeed_x_min) i_error_aspeed_x = i_error_aspeed_x_min;
	if(i_error_aspeed_y > i_error_aspeed_y_max) i_error_aspeed_y = i_error_aspeed_y_max;
	if(i_error_aspeed_y < i_error_aspeed_y_min) i_error_aspeed_y = i_error_aspeed_y_min;
	if(i_error_aspeed_z > i_error_aspeed_z_max) i_error_aspeed_z = i_error_aspeed_z_max;
	if(i_error_aspeed_z < i_error_aspeed_z_min) i_error_aspeed_z = i_error_aspeed_z_min;

	//瑙掗�搴﹁宸井鍒�
	d_error_aspeed_x = (error_aspeed_x - old_error_aspeed_x) / T;
	d_error_aspeed_y = (error_aspeed_y - old_error_aspeed_y) / T;
	d_error_aspeed_z = (error_aspeed_z - old_error_aspeed_z) / T;
	//鍐呯幆pid杈撳嚭
	control_2_x = kp_aspeed_x * error_aspeed_x + ki_aspeed_x * i_error_aspeed_x + kd_aspeed_x * d_error_aspeed_x;
	control_2_y = kp_aspeed_y * error_aspeed_y + ki_aspeed_y * i_error_aspeed_y + kd_aspeed_y * d_error_aspeed_y;
	control_2_z = kp_aspeed_z * error_aspeed_z + ki_aspeed_z * i_error_aspeed_z + kd_aspeed_z * d_error_aspeed_z;
	//淇濆瓨鍘嗗彶鏁版嵁
	old_error_aspeed_x = error_aspeed_x;
	old_error_aspeed_y = error_aspeed_y;
	old_error_aspeed_z = error_aspeed_z;
}
//涓�敭璧烽閫氳繃鏀瑰彉楂樺害璇樊绉垎甯告暟I瀹炵幇
//褰撶鍦颁箣鍓嶏紝I鍙栫殑姣旇緝澶э紙ki_height_ready锛夛紝褰撳垽鏂捣椋炰箣鍚庯紝I鍙栫殑姣旇緝灏忥紙ki_height锛�
void control_height()
{
		//楂樺害鐜�
		if(mode_land==0)
		{
			except_height=(CH_1-CH_1_MIN)/5.0f+0.4f;
		}
		//寰楀埌楂樺害璇樊
		error_height = except_height - US100_Alt;

		//楂樺害璇樊寰垎
		d_error_height = (error_height - old_error_height);
		if(US100_Alt<2.5f&&fly_low==0)
		{
			//楂樺害璇樊绉垎
			i_error_height = (i_error_height + ki_height_ready*error_height);
			fly_thr=0;
		}
		//璧烽楂樺害鍒ゆ柇涓�0cm
		if(US100_Alt>2.0f&&fly_low==0)
		{
			fly_low=1;
			fly_thr=i_error_height;
			i_error_height=0;
		}
		if(fly_low==1)
		{
			i_error_height = (i_error_height + ki_height*error_height);
		}
		//楂樺害璇樊绉垎闄愬箙
		if(i_error_height > 16000.0f) i_error_aspeed_x = 16000.0f;
		if(i_error_height < -16000.0f) i_error_aspeed_x = -16000.0f;
		//楂樺害鐜�
		if(mode_land==0)
		{
			ctrl_height_out = fly_thr+kp_height * error_height + i_error_height + kd_height * d_error_height;
		}
		else
		{
			ctrl_height_out = fly_thr+kp_height * error_height + i_error_height + 200.0f * d_error_height;
		}
		//淇濆瓨鍘嗗彶鏁版嵁
		old_error_height = error_height;
}
void land()//涓�敭闄嶈惤
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
//瀵逛簬ADNS3080鐨勬帶鍒跺畾鐐圭殑绋嬪簭锛岀敱浜�080绉垎璇樊寰堝ぇ锛屼笉瀹滈噰鐢ㄧН鍒嗘帶鍒�
void control_point()//瀹氱偣鎺у埗
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
void control_point_1()//瀹氱偣鎺у埗1
{
	if(game_count==0)
	{
		if(X1!=0)
		{
			error_X=X1-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
		if(Y1!=0)
		{
			error_Y=Y1-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==1)
	{
		game_angle_y=(float)game_speed/100;
		if(Y1!=0)
		{
			error_Y=Y1-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
		error_X=0;
	}
	else if(game_count==2)
	{
		if(X1<30&&shache_flag==0)
		{
			game_angle_y=shache/(-10);
			shache_flag=1;
		}
		else
		{
			game_angle_y=0;
		}
		if(X1!=0)
		{
			error_X=X1-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
		if(Y2!=0)
		{
			error_Y=Y2-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==3)
	{
		game_angle_x=(float)game_speed/150;
		if(X1!=0)
		{
			error_X=X1-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==4)
	{
		game_angle_x=0;
		if(Y2<40&&shache_flag_2==0)
		{
			game_angle_x=shache/(-10);
			shache_flag_2=1;
		}
		else
		{
			game_angle_x=0;
		}
		if(X1!=0)
		{
			error_X=X1-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
		if(Y2!=0)
		{
			error_Y=Y2-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==5)
	{
		game_angle_x=(float)game_speed/150;
		if(X1!=0)
		{
			error_X=X1-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==6)
	{
		game_angle_x=0;
		if(Y2<40&&shache_flag_3==0)
		{
			game_angle_x=shache/(-10);
			shache_flag_3=1;
		}
		else
		{
			game_angle_x=0;
		}
		if(X2!=0)
		{
			error_X=X2-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
		if(Y2!=0)
		{
			error_Y=Y2-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==7)
	{
		game_angle_y=(float)game_speed/(-150);
		if(Y2!=0)
		{
			error_Y=Y2-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
		error_X=0;
	}

	else if(game_count==8)
	{
		game_angle_y=0;
		if(X2>30&&shache_flag_4==0)
		{
			game_angle_y=shache/(10);
			shache_flag_4=1;
		}
		else
		{
			game_angle_y=0;
		}
		if(X2!=0)
		{
			error_X=X2-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
		if(Y1!=0)
		{
			error_Y=Y1-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==9)
	{
		game_angle_x=(float)game_speed/(-150);
		if(X2!=0)
		{
			error_X=X2-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==10)
	{
		game_angle_x=0;
		if(Y1<40&&shache_flag_5==0)
		{
			game_angle_x=shache/(10);
			shache_flag_5=1;
		}
		else
		{
			game_angle_x=0;
		}
		if(X2!=0)
		{
			error_X=X2-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
		if(Y1!=0)
		{
			error_Y=Y1-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==11)
	{
		game_angle_x=(float)game_speed/(-150);
		if(X2!=0)
		{
			error_X=X2-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
	}
	else if(game_count==12)
	{
		game_angle_x=0;
		if(Y1<40&&shache_flag_6==0)
		{
			game_angle_x=shache/(10);
			shache_flag_6=1;
		}
		else
		{
			game_angle_x=0;
		}
		if(X1!=0)
		{
			error_X=X1-30+(Q_ANGLE_Y-offset_angle_y)*buchang*US100_Alt/50;
		}
		if(Y1!=0)
		{
			error_Y=Y1-40+(Q_ANGLE_X-offset_angle_x)*buchang*US100_Alt/50;
		}
	}

	if(US100_Alt<1.5f)
	{
		error_X=0;
		error_Y=0;
	}

	/*if(error_vx>-2&&error_vx<2)
		error_vx=0;
	if(error_vy>-2&&error_vy<2)
		error_vy=0;*/
	d_error_X = (error_X - old_error_X);
	d_error_Y = (error_Y - old_error_Y);

	i_error_X+=error_X;
	i_error_Y+=error_Y;
	except_x_angle=kp_point*0.001f*error_X+kd_point*0.001f*d_error_X+ki_point*0.0001f*i_error_X;
	except_y_angle=kp_point*0.001f*error_Y+kd_point*0.001f*d_error_Y+ki_point*0.0001f*i_error_Y;

	if(except_x_angle>3.0f) except_x_angle=3.0f;
	if(except_x_angle<-3.0f) except_x_angle=-3.0f;
	if(except_y_angle>3.0f) except_y_angle=3.0f;
	if(except_y_angle<-3.0f) except_y_angle=-3.0f;

	old_error_X=error_X;
	old_error_Y=error_Y;
}
void control()
{
	except_angle_x=(CH_5-CH_5_MID)/1.0f*(-1)-except_y_angle+game_angle_x;
	except_angle_y=(CH_6-CH_6_MID)/1.0f-except_x_angle+game_angle_y;
	except_aspeed_z=(CH_2-CH_2_MID)*4.0f*(-1);//z轴控制角速度
	control_1();//澶栫幆
	control_2();//鍐呯幆
	if(CH_1<CH_1_MIN||CH_1>CH_1_MAX)//娌归棬寮傚父鍊兼帓闄�
	{
		CH_1=CH_1_MIN;
	}

	if(CH_3<CH_3_HEIGHT+5&&CH_3>CH_3_HEIGHT-5)//瑙ｉ攣锛屽閿�0锛岃緟鍔╁紑鍏虫墦鍒版渶楂樿В閿侊紝鍚屾椂z杞村紑濮嬬Н鍒�
	{
		/////瀹氶珮妯″紡
		mode_height=1;//寮�惎瀹氶珮妯″紡
		motor_1 = 20000 + ctrl_height_out - control_1_x - control_1_y - control_1_z + control_2_x - control_2_y + control_2_z;
		motor_2 = 20000 + ctrl_height_out + control_1_x - control_1_y + control_1_z - control_2_x - control_2_y - control_2_z;
		motor_3 = 20000 + ctrl_height_out + control_1_x + control_1_y - control_1_z - control_2_x + control_2_y + control_2_z;
		motor_4 = 20000 + ctrl_height_out - control_1_x + control_1_y + control_1_z + control_2_x + control_2_y - control_2_z;
	}
	else if(CH_3<CH_3_HAND+5&&CH_3>CH_3_HAND-5)//瑙ｉ攣锛屽閿�0锛岃緟鍔╁紑鍏虫墦鍒版渶楂樿В閿侊紝鍚屾椂z杞村紑濮嬬Н鍒�
	{
		/////鎵嬪姩妯″紡
		if(mode_height==0)
		{
			motor_1 = 20000 +(CH_1 - CH_1_MIN) * 200 - control_1_x - control_1_y - control_1_z + control_2_x - control_2_y + control_2_z;
			motor_2 = 20000 +(CH_1 - CH_1_MIN) * 200 + control_1_x - control_1_y + control_1_z - control_2_x - control_2_y - control_2_z;
			motor_3 = 20000 +(CH_1 - CH_1_MIN) * 200 + control_1_x + control_1_y - control_1_z - control_2_x + control_2_y + control_2_z;
			motor_4 = 20000 +(CH_1 - CH_1_MIN) * 200 - control_1_x + control_1_y + control_1_z + control_2_x + control_2_y - control_2_z;

		}
		else if(mode_height==1)//瀹氶珮鐨勬椂鍊欐墦寮�繖涓紑鍏充唬琛ㄥ紑鍚檷钀芥ā寮�
		{
			mode_land=1;//寮�惎闄嶈惤妯″紡
			motor_1 = 20000 + ctrl_height_out - control_1_x - control_1_y - control_1_z + control_2_x - control_2_y + control_2_z;
			motor_2 = 20000 + ctrl_height_out + control_1_x - control_1_y + control_1_z - control_2_x - control_2_y - control_2_z;
			motor_3 = 20000 + ctrl_height_out + control_1_x + control_1_y - control_1_z - control_2_x + control_2_y + control_2_z;
			motor_4 = 20000 + ctrl_height_out - control_1_x + control_1_y + control_1_z + control_2_x + control_2_y - control_2_z;

		}
		//
	}
	else //鏈В閿侊紝闄愬埗pwm杈撳嚭銆亃杞磋閫熷害
	{
		motor_1 = 20000;
		motor_2 = 20000;
		motor_3 = 20000;
		motor_4 = 20000;
		GYRO_I_Z=0;//闄�灪浠Н鍒嗘竻闆�
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
	}
	if(CH_1<CH_1_MIN+2)//娌归棬鎵撳埌鏈�綆绂佹杈撳嚭
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
		error_flag=1;//鍏抽棴鐢垫満
		mode_land=0;
		mode_height=0;
	}
	if(Q_ANGLE_X>50 || Q_ANGLE_X<-50 || Q_ANGLE_Y>50 || Q_ANGLE_Y<-50 )
	{
		error_flag=1;
	}
	if(error_flag==1)//瑙掑害鍑虹幇寮傚父鍊�
	{
		motor_1 = 20000;
		motor_2 = 20000;
		motor_3 = 20000;
		motor_4 = 20000;
	}

	//pwm娉㈤檺骞咃紝20000瀵瑰簲1ms锛�0000瀵瑰簲2ms
	if(motor_1<20000) motor_1=20000;
	if(motor_1>40000) motor_1=40000;
	if(motor_2<20000) motor_2=20000;
	if(motor_2>40000) motor_2=40000;
	if(motor_3<20000) motor_3=20000;
	if(motor_3>40000) motor_3=40000;
	if(motor_4<20000) motor_4=20000;
	if(motor_4>40000) motor_4=40000;
	//鍒锋柊pwm鍊�
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, motor_2);//m2
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, motor_1);//m1
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, motor_4);//m4
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, motor_3);//m3
}
