/*
 * PWM.h
 *
 *  Created on: 2015-11-13
 *      Author: lenovo-pc
 */

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#define GPIO_PE5_M0PWM5         0x00041404
#define GPIO_PE5_M1PWM3         0x00041405

#define GPIO_PE4_M0PWM4         0x00041004
#define GPIO_PE4_M1PWM2         0x00041005

#define GPIO_PD1_M0PWM7         0x00030404
#define GPIO_PD1_M1PWM1         0x00030405

#define GPIO_PD0_M0PWM6         0x00030004
#define GPIO_PD0_M1PWM0         0x00030005

void pwm_init();

#endif /* PWM_H_ */
