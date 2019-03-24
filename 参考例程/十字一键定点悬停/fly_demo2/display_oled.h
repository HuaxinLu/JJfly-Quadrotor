/*********************************************************************************
JJ_fly by TIVA
文件名：display_oled.h
描述：OLED显示，软件SPI
作者：卢华歆
时间：2015.10
**********************************************************************************/

#ifndef DISPLAY_OLED_H_
#define DISPLAY_OLED_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"

#define  u8 unsigned char
#define  u32 unsigned int
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

#define OLED_CLK_GROUP GPIO_PORTD_BASE
#define OLED_CLK_NUM GPIO_PIN_0
#define OLED_MOSI_GROUP GPIO_PORTD_BASE
#define OLED_MOSI_NUM GPIO_PIN_1
#define OLED_RES_GROUP GPIO_PORTD_BASE
#define OLED_RES_NUM GPIO_PIN_2
#define OLED_DC_GROUP GPIO_PORTD_BASE
#define OLED_DC_NUM GPIO_PIN_3
#define OLED_CS_GROUP GPIO_PORTE_BASE
#define OLED_CS_NUM GPIO_PIN_1


#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF
#define X_WIDTH 	128
#define Y_WIDTH 	64
//-----------------OLED端口定义----------------

void delay_ms(unsigned int ms);

void OLED_SCLK_Set();
void OLED_SCLK_Clr();
void OLED_SDIN_Set();
void OLED_SDIN_Clr();
void OLED_DC_Set()	;
void OLED_DC_Clr() ;
void OLED_CS_Set()	;
void OLED_CS_Clr()	;
void OLED_RST_Set();
void OLED_RST_Clr();

//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2);
void OLED_ShowCharfan(u8 x,u8 y,u8 chr);
void OLED_ShowNumfan(u8 x,u8 y,u32 num,u8 len,u8 size2);
void OLED_ShowString(u8 x,u8 y, u8 *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);

#endif /* DISPLAY_OLED_H_ */
