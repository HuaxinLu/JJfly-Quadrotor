/*********************************************************************************
JJ_fly by TIVA
文件名：flow.c
描述：ADNS3080数据读取与处理
作者：卢华歆
时间：2016.4
**********************************************************************************/
#include "PIN.h"
#include "flow.h"
#include "mymath.h"
unsigned char led_flag;
char vx,vy;
char vx_temp;
char vy_temp;
unsigned char flow_count;
unsigned char img[5];
unsigned char camera_count;
char flow_flag;
char flow_data[10];
unsigned char X1,Y1,X2,Y2;
void read_flow();
void read_camera();
void flow_init()//串口初始化
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//使能uart0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//开外设使能
	GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);//按键引脚设置为输入
	GPIOPinConfigure(GPIO_PE1_U7TX);
	GPIOPinConfigure(GPIO_PE0_U7RX);
	GPIOPinTypeUART(GPIO_PORTE_BASE,GPIO_PIN_0|GPIO_PIN_1);//使能相应IO
	//uart0,波特率115200,8位数据位1位停止位
	UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTFIFOEnable(UART7_BASE);
	UARTFIFOLevelSet(UART7_BASE,UART_FIFO_TX7_8,UART_FIFO_RX2_8);//发送FIFO达到7/8时中断，接收FIFO达到6/8时中断
	UARTIntRegister(UART7_BASE,read_flow);
    IntMasterEnable();//开启总中断
    IntEnable(INT_UART7_SNOWFLAKE);//开启uart0中断
    UARTIntEnable(UART7_BASE, UART_INT_RX);//仅开启RX中断
}
void k60_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);//使能uart0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//开外设使能
	GPIOPinConfigure(GPIO_PC4_U4RX);
	GPIOPinConfigure(GPIO_PC5_U4TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5);//使能相应IO
	//uart0,波特率115200,8位数据位1位停止位
	UARTConfigSetExpClk(UART4_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTFIFOLevelSet(UART4_BASE,UART_FIFO_TX7_8,UART_FIFO_RX4_8);//发送FIFO达到7/8时中断，接收FIFO达到6/8时中断
	UARTIntRegister(UART4_BASE,read_camera);
	IntMasterEnable();//开启总中断
    IntEnable(INT_UART4_SNOWFLAKE);//开启uart0中断
    UARTIntEnable(UART4_BASE, UART_INT_RX);//仅开启RX中断
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_2);
}
void read_flow()//串口中断（不知道为什么在STARTUP中无法注册中断向量）
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART7_BASE, true);

    UARTIntClear(UART7_BASE, ui32Status);
    while(UARTCharsAvail(UART7_BASE))
    {
    	flow_data[flow_count]=UARTCharGetNonBlocking(UART7_BASE);
    	if(flow_data[0]==0xAA)//帧头AA BB
    	{
    		flow_count++;
    		if(flow_count==4)
    		{
    			flow_count=0;
    			if(flow_data[1]==0xBB)
    			{
    				vx_temp=flow_data[2]+127;//原始数据是-127—+127便于计算加127进行转换
    				vy_temp=flow_data[3]+127;
    				flow_flag=1;

    			}
    			flow_data[0]=0;
    		}
    	}
    }
}

char flow_vx_num,flow_vy_num;

char send_flag;
char FLOW_NUM=5;
int VX_BUF[10];
int VY_BUF[10];
int vx_lpf;
int vy_lpf;
char vx_his;
char vy_his;
//光流原始数据毛刺很多，必须要滤波
//该滤波采用滑动滤波，掐头去尾取平均，再滤除一些毛刺
//该效果不够理想，还需改进
void flow_filter()
{
	char i;
	int temp1=0;
	int temp2=0;
	int max1,min1;
	int max2,min2;
	char vx_avg,vy_avg;
	VX_BUF[flow_vx_num] = vx_temp;//更新滑动窗口数组
	VY_BUF[flow_vx_num] = vy_temp;//更新滑动窗口数组
	max1=VX_BUF[0];
	min1=VX_BUF[0];
	max2=VY_BUF[0];
	min2=VY_BUF[0];
	for(i=0;i<FLOW_NUM;i++)//寻找两个数组中的最大与最小值
	{
		if(VX_BUF[i]>max1) max1=VX_BUF[i];
		if(VX_BUF[i]<min1) min1=VX_BUF[i];
		if(VY_BUF[i]>max2) max2=VY_BUF[i];
		if(VY_BUF[i]<min2) max2=VY_BUF[i];
		temp1+=VX_BUF[i];
		temp2+=VY_BUF[i];
	}
	vx_avg = (temp1-max1-min1) / (FLOW_NUM-2);
	vy_avg = (temp2-max2-min2) / (FLOW_NUM-2);//中间数据求评价
	//Y+=(vy-127);
	if(vx_avg-vx_his<10&&vx_avg-vx_his>-10||vx_avg==127)//滤除毛刺数据
	{
			vx=vx_avg;
		    //X+=(vx-127);
			vx_his=vx;//保存历史数据
	}


	if(vy_avg-vy_his<10&&vy_avg-vy_his>-10||vy_avg==127)
	{
		    vy=vy_avg;
		    //Y+=(vy-127);
			vy_his=vy;
	}

	flow_vx_num++;
	if(flow_vx_num==FLOW_NUM)
	{
		flow_vx_num=0;
	}
}
void read_camera()
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART4_BASE, true);

    UARTIntClear(UART4_BASE, ui32Status);
    while(UARTCharsAvail(UART4_BASE))
    {
    	img[camera_count]=UARTCharGetNonBlocking(UART4_BASE);
    	if(img[0]==0xAA)//帧头AA BB
    	{
    		camera_count++;
    		if(camera_count==5)
    		{
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,led_flag);
				led_flag=0xff-led_flag;
    			camera_count=0;
    			X1=img[1];
    			Y1=img[2];
    			X2=img[3];
    			Y2=img[4];
    			img[0]=0;
    		}
    	}
    }
}
