#ifndef __HONGWAI_H
#define __HONGWAI_H

#include "stm32f4xx.h"


// 声明全局变量（在 hongwai.c 中定义）
extern volatile uint8_t avoidRightActive;
extern volatile uint8_t avoidLeftActive;
extern volatile uint8_t avoidForwardRightActive;
extern volatile uint8_t avoidForwardLeftActive;

// 重新定义引脚宏（若头文件不可修改，在此覆盖定义）
#define INFRAREDAVOID1_INT_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define INFRAREDAVOID1_INT_GPIO_PORT    GPIOD
#define INFRAREDAVOID1_INT_GPIO_PIN     GPIO_Pin_8//右边
#define INFRAREDAVOID1_INT_EXTI_PortSource EXTI_PortSourceGPIOD
#define INFRAREDAVOID1_INT_EXTI_PinSource  EXTI_PinSource8
#define INFRAREDAVOID1_INT_EXTI_LINE     EXTI_Line8

#define INFRAREDAVOID2_INT_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define INFRAREDAVOID2_INT_GPIO_PORT    GPIOD
#define INFRAREDAVOID2_INT_GPIO_PIN     GPIO_Pin_9//左边
#define INFRAREDAVOID2_INT_EXTI_PortSource EXTI_PortSourceGPIOD
#define INFRAREDAVOID2_INT_EXTI_PinSource  EXTI_PinSource9
#define INFRAREDAVOID2_INT_EXTI_LINE     EXTI_Line9

#define INFRAREDAVOID3_INT_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define INFRAREDAVOID3_INT_GPIO_PORT    GPIOD
#define INFRAREDAVOID3_INT_GPIO_PIN     GPIO_Pin_7//右前
#define INFRAREDAVOID3_INT_EXTI_PortSource EXTI_PortSourceGPIOD
#define INFRAREDAVOID3_INT_EXTI_PinSource  EXTI_PinSource7
#define INFRAREDAVOID3_INT_EXTI_LINE     EXTI_Line7

#define INFRAREDAVOID4_INT_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define INFRAREDAVOID4_INT_GPIO_PORT    GPIOD
#define INFRAREDAVOID4_INT_GPIO_PIN     GPIO_Pin_6//左前
#define INFRAREDAVOID4_INT_EXTI_PortSource EXTI_PortSourceGPIOD
#define INFRAREDAVOID4_INT_EXTI_PinSource  EXTI_PinSource6
#define INFRAREDAVOID4_INT_EXTI_LINE     EXTI_Line6


// 函数声明
void EXTI_INFRAREDAVOID1_Config(void);
void EXTI_INFRAREDAVOID2_Config(void);
void EXTI_INFRAREDAVOID3_Config(void);
void EXTI_INFRAREDAVOID4_Config(void);

void EXTI9_5_IRQHandler(void);
#endif /* __BSP_EXTI_H */

