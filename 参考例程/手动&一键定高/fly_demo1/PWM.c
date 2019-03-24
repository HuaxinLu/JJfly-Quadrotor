/*
 * PWM.c
 *
 *  Created on: 2015-11-13
 *      Author: lenovo-pc
 */


#include "PWM.h"

void pwm_init()
{
    //PWM时钟配置：64分频
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
    //使能PWM1模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //使能PWM1模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //使能PWM引脚
    GPIOPinConfigure(GPIO_PE5_M1PWM3);
    GPIOPinConfigure(GPIO_PE4_M1PWM2);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    //配置引脚为PWM功能
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    //配置PWM发生器0和发生器1：加减计数不分频
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //设置PWM发生器1的频率，时钟频率/pwm分频数/n,80M/64/3125=400hz
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 50000);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 50000);

    //设置PWM1/PWM1输出的脉冲宽度
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 20000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 20000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 20000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 20000);

    //使能PWM发生器
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);

    //使能PWM1和PWM1的输出
    PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT), true);
    PWMOutputState(PWM1_BASE, (PWM_OUT_1_BIT), true);
    PWMOutputState(PWM1_BASE, (PWM_OUT_2_BIT), true);
    PWMOutputState(PWM1_BASE, (PWM_OUT_3_BIT), true);
}



