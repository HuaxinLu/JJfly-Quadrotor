/*********************************************************************************
JJ_fly by TIVA
文件名：nrf2401.c
描述：nrf2401的发送程序
作者：卢华歆
时间：2015.10
**********************************************************************************/
#include "nrf2401.h"
#include "mpu6050.h"
#include "uarts.h"
#include "imu_new.h"
#include "SPI.h"
#include "capture.h"
uint8_t TX_ADDRESS[TX_ADR_WIDTH]  = {0x34,0x43,0x10,0x10,0x01};//设置发送地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH]  = {0x34,0x43,0x10,0x10,0x01};//设置接收地址
uint8_t TX_Buf[TX_PLOAD_WIDTH]={0x01,0x02,0x03,0x4,0x05,0x06,0x07,0x08,
								0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,
								0x17,0x18,0x19,0x20,0x21,0x22,0x23,0x24,
								0x25,0x26,0x27,0x28,0x29,0x30,0x31,0x32,};//发送数据表
uint8_t RX_Buf[TX_PLOAD_WIDTH];//接收数据
//*********************************************************************
void nrf24l01init(void)
{
    GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_5,0);    // 处于待机模式一，所以可以写寄存器
    //SPI_Write_register(FLUSH_TX,0x00);
    SPI_Write_register(WRITE_REG+EN_AA,0X01);//失能通道0自动应答,方便单独调试
    SPI_Write_register(WRITE_REG+EN_RXADDR,0X01);//失能接受通道0，方便单独调试发送模块
    //SPI_Write_register(WRITE_REG+SETUP_RETR,0x00);//失能自动重发
    SPI_Write_register(WRITE_REG + RF_CH, 0x00);        // Select RF channel 40
    SPI_Write_register(WRITE_REG +RX_PW_P0,RX_PLOAD_WIDTH);
    SPI_Write_register(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
    SPI_Write_register(WRITE_REG+STATUS,0xff);

    SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
    //SPI_Write_register(FLUSH_TX,0x00);

    SPI_Write_register(WRITE_REG + CONFIG1, 0x0e);


    GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_5,GPIO_PIN_5);


    SysCtlDelay(235);//
}
//*************************************************************************
void nrf24l01TxMode(void)//参考功能使用文档设置
{
    GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_5,0);
    SPI_Write_Buf(WR_TX_PLOAD, TX_Buf, TX_PLOAD_WIDTH); // Writes dat
    GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_5,GPIO_PIN_5);
    SysCtlDelay(200);//10us多点
    GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_5,0);
}

void nrf2401_send_Status()
{
	int _temp;
	unsigned char i;
	unsigned char _cnt=0;

	TX_Buf[_cnt++]=0xAA;
	TX_Buf[_cnt++]=0xAA;
	TX_Buf[_cnt++]=0x01;

    TX_Buf[_cnt++]=0;

	_temp = (int)(Q_ANGLE_X*100);
	TX_Buf[_cnt++]=BYTE1(_temp);
	TX_Buf[_cnt++]=BYTE0(_temp);
	_temp = (int)(Q_ANGLE_Y*100);
	TX_Buf[_cnt++]=BYTE1(_temp);
	TX_Buf[_cnt++]=BYTE0(_temp);
	_temp = (int)(Q_ANGLE_Z*10);
	TX_Buf[_cnt++]=BYTE1(_temp);
	TX_Buf[_cnt++]=BYTE0(_temp);

	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;

	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;

	TX_Buf[3]=_cnt-4;
	unsigned char sum = 0;
	for(i=0;i<_cnt;i++)
		sum += TX_Buf[i];
	TX_Buf[_cnt++]=sum;
	nrf24l01TxMode();
}

void nrf2401_send_RCData()
{
	unsigned char i;
	unsigned char _cnt=0;

	TX_Buf[_cnt++]=0xAA;
	TX_Buf[_cnt++]=0xAA;
	TX_Buf[_cnt++]=0x03;

    TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=BYTE1(CH_1);
	TX_Buf[_cnt++]=BYTE0(CH_1);
	TX_Buf[_cnt++]=BYTE1(CH_2);
	TX_Buf[_cnt++]=BYTE0(CH_2);
	TX_Buf[_cnt++]=BYTE1(CH_5);
	TX_Buf[_cnt++]=BYTE0(CH_5);
	TX_Buf[_cnt++]=BYTE1(CH_6);
	TX_Buf[_cnt++]=BYTE0(CH_6);
	TX_Buf[_cnt++]=BYTE1(CH_3);
	TX_Buf[_cnt++]=BYTE0(CH_3);
	TX_Buf[_cnt++]=BYTE1(CH_4);
	TX_Buf[_cnt++]=BYTE0(CH_4);

	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;

	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;
	TX_Buf[_cnt++]=0;

	TX_Buf[3]=_cnt-4;
	unsigned char sum = 0;
	for(i=0;i<_cnt;i++)
		sum += TX_Buf[i];
	TX_Buf[_cnt++]=sum;
	nrf24l01TxMode();
}
