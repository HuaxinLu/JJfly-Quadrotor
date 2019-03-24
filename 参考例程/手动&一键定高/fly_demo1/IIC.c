/*********************************************************************************
JJ_fly by TIVA
文件名：IIC.c
描述：硬件IIC
作者：卢华歆
时间：2015.10
**********************************************************************************/
#include "IIC.h"
unsigned char SLAVE_ADDRESS;
void i2c_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);// 使能I2C2模块的时钟信号
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//开外设使能

	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);// 将PB2的功能选择为I2C2模块的时钟信号，将PB3选择为数据信号

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);// 对PB2和PB3两个引脚做有关I2C功能的配置

    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet()/2, true);

    I2CMasterEnable(I2C1_BASE);
}
unsigned char i2c_write(unsigned char reg_addr,unsigned char data)//IIC写寄存器
{
    //设置I2C2主模块将要放在总线上的从模块地址
    //false代表主模块发送，从模块接收
	I2CMasterSlaveAddrSet(I2C1_BASE,SLAVE_ADDRESS, false);
	I2CMasterDataPut(I2C1_BASE, reg_addr);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C1_BASE));
	// 主模块开始发送寄存器地址
	I2CMasterDataPut(I2C1_BASE, data);
	// 主模块开始发送数据
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	while(I2CMasterBusy(I2C1_BASE));
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
	while(I2CMasterBusy(I2C1_BASE));
	/*I2CMasterDataPut(I2C2_BASE, data);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C2_BASE));*/
	return 0;
}
unsigned char i2c_read(unsigned char reg_addr)//IIC读寄存器
{
    I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, false);
    I2CMasterDataPut(I2C1_BASE, reg_addr);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C1_BASE));
    I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, true);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(I2C1_BASE));
    return I2CMasterDataGet(I2C1_BASE);
}
int Get16Bit(unsigned char reg_addr)
{
	  unsigned char ho,lo;
	  int temp;
	  ho = i2c_read(reg_addr);
	  lo = i2c_read(reg_addr+1);
	  temp=(ho<<8)+lo;
	  if(temp>=32768)
		  return -1*(65536-temp);
	  else
		  return temp;
}
