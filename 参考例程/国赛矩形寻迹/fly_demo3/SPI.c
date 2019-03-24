/*********************************************************************************
JJ_fly by TIVA
文件名：SPI.c
描述：硬件SPI
作者：卢华歆
时间：2015.10
**********************************************************************************/
#include "SPI.h"
void SPI_TM4C123_init(void)
{

    uint32_t ui32Data;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet()*2, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,SSI_BitRate,8);
    SSIEnable(SSI0_BASE);
    while(SSIDataGetNonBlocking(SSI0_BASE, &ui32Data)){}
}
//
uint8_t SPI_RW(uint8_t value)
{
    uint32_t ui32Data;
    uint8_t ui8Data;
    SSIDataPut(SSI0_BASE,value);
    while (SSIBusy(SSI0_BASE)){}
    SSIDataGet(SSI0_BASE, &ui32Data);
    ui8Data = ui32Data & 0xff;
    return(ui8Data);
}
//********************************************************************
//用SPI配置nrf24l01的寄存器函数
uint8_t SPI_Write_register(uint8_t WriteReg,uint8_t Value)//WriteReg要写的寄存器地址，Value要写的寄存器值
{
    uint8_t status;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,0);
    status=SPI_RW(WriteReg);
    SPI_RW(Value);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_PIN_3);
    return(status);
}
//**********************************************************************
uint8_t SPI_Write_Buf(uint8_t WriteReg, uint8_t *pBuf, uint8_t bytes)
{
    uint8_t i,status;
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,0);
    status=SPI_RW(WriteReg);
    for(i=0; i<bytes; i++) // then write all byte in buffer(*pBuf)
    {
        SPI_RW(*pBuf++);
    }
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_PIN_3);
    return(status);
}
//*************************************************************************
uint8_t SPIReadStatus(uint8_t reg)
{
    uint8_t status;
    SysCtlDelay(100);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,0);
    SPI_RW(reg);
    status=SPI_RW(0);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_PIN_3);
    return(status);
}
//***********************************************************************

