
#ifndef PIN_H_
#define PIN_H_

//*****************************************************************************
//
// TM4C123GH6PM Port/Pin Mapping Definitions
//
//*****************************************************************************

#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA0_CAN1RX         0x00000008

#define GPIO_PA1_U0TX           0x00000401
#define GPIO_PA1_CAN1TX         0x00000408

#define GPIO_PA2_SSI0CLK        0x00000802

#define GPIO_PA3_SSI0FSS        0x00000C02

#define GPIO_PA4_SSI0RX         0x00001002

#define GPIO_PA5_SSI0TX         0x00001402

#define GPIO_PA6_I2C1SCL        0x00001803
#define GPIO_PA6_M1PWM2         0x00001805

#define GPIO_PA7_I2C1SDA        0x00001C03
#define GPIO_PA7_M1PWM3         0x00001C05

#define GPIO_PB0_U1RX           0x00010001
#define GPIO_PB0_T2CCP0         0x00010007

#define GPIO_PB1_U1TX           0x00010401
#define GPIO_PB1_T2CCP1         0x00010407

#define GPIO_PB2_I2C0SCL        0x00010803
#define GPIO_PB2_T3CCP0         0x00010807

#define GPIO_PB3_I2C0SDA        0x00010C03
#define GPIO_PB3_T3CCP1         0x00010C07

#define GPIO_PB4_SSI2CLK        0x00011002
#define GPIO_PB4_M0PWM2         0x00011004
#define GPIO_PB4_T1CCP0         0x00011007
#define GPIO_PB4_CAN0RX         0x00011008

#define GPIO_PB5_SSI2FSS        0x00011402
#define GPIO_PB5_M0PWM3         0x00011404
#define GPIO_PB5_T1CCP1         0x00011407
#define GPIO_PB5_CAN0TX         0x00011408

#define GPIO_PB6_SSI2RX         0x00011802
#define GPIO_PB6_M0PWM0         0x00011804
#define GPIO_PB6_T0CCP0         0x00011807

#define GPIO_PB7_SSI2TX         0x00011C02
#define GPIO_PB7_M0PWM1         0x00011C04
#define GPIO_PB7_T0CCP1         0x00011C07

#define GPIO_PC0_TCK            0x00020001
#define GPIO_PC0_SWCLK          0x00020001
#define GPIO_PC0_T4CCP0         0x00020007

#define GPIO_PC1_TMS            0x00020401
#define GPIO_PC1_SWDIO          0x00020401
#define GPIO_PC1_T4CCP1         0x00020407

#define GPIO_PC2_TDI            0x00020801
#define GPIO_PC2_T5CCP0         0x00020807

#define GPIO_PC3_SWO            0x00020C01
#define GPIO_PC3_TDO            0x00020C01
#define GPIO_PC3_T5CCP1         0x00020C07

#define GPIO_PC4_U4RX           0x00021001
#define GPIO_PC4_U1RX           0x00021002
#define GPIO_PC4_M0PWM6         0x00021004
#define GPIO_PC4_IDX1           0x00021006
#define GPIO_PC4_WT0CCP0        0x00021007
#define GPIO_PC4_U1RTS          0x00021008

#define GPIO_PC5_U4TX           0x00021401
#define GPIO_PC5_U1TX           0x00021402
#define GPIO_PC5_M0PWM7         0x00021404
#define GPIO_PC5_PHA1           0x00021406
#define GPIO_PC5_WT0CCP1        0x00021407
#define GPIO_PC5_U1CTS          0x00021408

#define GPIO_PC6_U3RX           0x00021801
#define GPIO_PC6_PHB1           0x00021806
#define GPIO_PC6_WT1CCP0        0x00021807
#define GPIO_PC6_USB0EPEN       0x00021808

#define GPIO_PC7_U3TX           0x00021C01
#define GPIO_PC7_WT1CCP1        0x00021C07
#define GPIO_PC7_USB0PFLT       0x00021C08

#define GPIO_PD0_SSI3CLK        0x00030001
#define GPIO_PD0_SSI1CLK        0x00030002
#define GPIO_PD0_I2C3SCL        0x00030003
#define GPIO_PD0_M0PWM6         0x00030004
#define GPIO_PD0_M1PWM0         0x00030005
#define GPIO_PD0_WT2CCP0        0x00030007

#define GPIO_PD1_SSI3FSS        0x00030401
#define GPIO_PD1_SSI1FSS        0x00030402
#define GPIO_PD1_I2C3SDA        0x00030403
#define GPIO_PD1_M0PWM7         0x00030404
#define GPIO_PD1_M1PWM1         0x00030405
#define GPIO_PD1_WT2CCP1        0x00030407

#define GPIO_PD2_SSI3RX         0x00030801
#define GPIO_PD2_SSI1RX         0x00030802
#define GPIO_PD2_M0FAULT0       0x00030804
#define GPIO_PD2_WT3CCP0        0x00030807
#define GPIO_PD2_USB0EPEN       0x00030808

#define GPIO_PD3_SSI3TX         0x00030C01
#define GPIO_PD3_SSI1TX         0x00030C02
#define GPIO_PD3_IDX0           0x00030C06
#define GPIO_PD3_WT3CCP1        0x00030C07
#define GPIO_PD3_USB0PFLT       0x00030C08

#define GPIO_PD4_U6RX           0x00031001
#define GPIO_PD4_WT4CCP0        0x00031007

#define GPIO_PD5_U6TX           0x00031401
#define GPIO_PD5_WT4CCP1        0x00031407

#define GPIO_PD6_U2RX           0x00031801
#define GPIO_PD6_M0FAULT0       0x00031804
#define GPIO_PD6_PHA0           0x00031806
#define GPIO_PD6_WT5CCP0        0x00031807

#define GPIO_PD7_U2TX           0x00031C01
#define GPIO_PD7_PHB0           0x00031C06
#define GPIO_PD7_WT5CCP1        0x00031C07
#define GPIO_PD7_NMI            0x00031C08

#define GPIO_PE0_U7RX           0x00040001

#define GPIO_PE1_U7TX           0x00040401

#define GPIO_PE4_U5RX           0x00041001
#define GPIO_PE4_I2C2SCL        0x00041003
#define GPIO_PE4_M0PWM4         0x00041004
#define GPIO_PE4_M1PWM2         0x00041005
#define GPIO_PE4_CAN0RX         0x00041008

#define GPIO_PE5_U5TX           0x00041401
#define GPIO_PE5_I2C2SDA        0x00041403
#define GPIO_PE5_M0PWM5         0x00041404
#define GPIO_PE5_M1PWM3         0x00041405
#define GPIO_PE5_CAN0TX         0x00041408

#define GPIO_PF0_U1RTS          0x00050001
#define GPIO_PF0_SSI1RX         0x00050002
#define GPIO_PF0_CAN0RX         0x00050003
#define GPIO_PF0_M1PWM4         0x00050005
#define GPIO_PF0_PHA0           0x00050006
#define GPIO_PF0_T0CCP0         0x00050007
#define GPIO_PF0_NMI            0x00050008
#define GPIO_PF0_C0O            0x00050009

#define GPIO_PF1_U1CTS          0x00050401
#define GPIO_PF1_SSI1TX         0x00050402
#define GPIO_PF1_M1PWM5         0x00050405
#define GPIO_PF1_PHB0           0x00050406
#define GPIO_PF1_T0CCP1         0x00050407
#define GPIO_PF1_C1O            0x00050409
#define GPIO_PF1_TRD1           0x0005040E

#define GPIO_PF2_SSI1CLK        0x00050802
#define GPIO_PF2_M0FAULT0       0x00050804
#define GPIO_PF2_M1PWM6         0x00050805
#define GPIO_PF2_T1CCP0         0x00050807
#define GPIO_PF2_TRD0           0x0005080E

#define GPIO_PF3_SSI1FSS        0x00050C02
#define GPIO_PF3_CAN0TX         0x00050C03
#define GPIO_PF3_M1PWM7         0x00050C05
#define GPIO_PF3_T1CCP1         0x00050C07
#define GPIO_PF3_TRCLK          0x00050C0E

#define GPIO_PF4_M1FAULT0       0x00051005
#define GPIO_PF4_IDX0           0x00051006
#define GPIO_PF4_T2CCP0         0x00051007
#define GPIO_PF4_USB0EPEN       0x00051008



#endif /* PIN_H_ */
