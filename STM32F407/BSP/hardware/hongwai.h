#ifndef __HONGWAI_H
#define __HONGWAI_H

#include "stm32f4xx.h"


// ����ȫ�ֱ������� hongwai.c �ж��壩
extern volatile uint8_t avoidRightActive;
extern volatile uint8_t avoidLeftActive;
extern volatile uint8_t avoidForwardRightActive;
extern volatile uint8_t avoidForwardLeftActive;

// ���¶������ź꣨��ͷ�ļ������޸ģ��ڴ˸��Ƕ��壩
#define INFRAREDAVOID1_INT_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define INFRAREDAVOID1_INT_GPIO_PORT    GPIOD
#define INFRAREDAVOID1_INT_GPIO_PIN     GPIO_Pin_8//�ұ�
#define INFRAREDAVOID1_INT_EXTI_PortSource EXTI_PortSourceGPIOD
#define INFRAREDAVOID1_INT_EXTI_PinSource  EXTI_PinSource8
#define INFRAREDAVOID1_INT_EXTI_LINE     EXTI_Line8

#define INFRAREDAVOID2_INT_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define INFRAREDAVOID2_INT_GPIO_PORT    GPIOD
#define INFRAREDAVOID2_INT_GPIO_PIN     GPIO_Pin_9//���
#define INFRAREDAVOID2_INT_EXTI_PortSource EXTI_PortSourceGPIOD
#define INFRAREDAVOID2_INT_EXTI_PinSource  EXTI_PinSource9
#define INFRAREDAVOID2_INT_EXTI_LINE     EXTI_Line9

#define INFRAREDAVOID3_INT_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define INFRAREDAVOID3_INT_GPIO_PORT    GPIOD
#define INFRAREDAVOID3_INT_GPIO_PIN     GPIO_Pin_7//��ǰ
#define INFRAREDAVOID3_INT_EXTI_PortSource EXTI_PortSourceGPIOD
#define INFRAREDAVOID3_INT_EXTI_PinSource  EXTI_PinSource7
#define INFRAREDAVOID3_INT_EXTI_LINE     EXTI_Line7

#define INFRAREDAVOID4_INT_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define INFRAREDAVOID4_INT_GPIO_PORT    GPIOD
#define INFRAREDAVOID4_INT_GPIO_PIN     GPIO_Pin_6//��ǰ
#define INFRAREDAVOID4_INT_EXTI_PortSource EXTI_PortSourceGPIOD
#define INFRAREDAVOID4_INT_EXTI_PinSource  EXTI_PinSource6
#define INFRAREDAVOID4_INT_EXTI_LINE     EXTI_Line6


// ��������
void EXTI_INFRAREDAVOID1_Config(void);
void EXTI_INFRAREDAVOID2_Config(void);
void EXTI_INFRAREDAVOID3_Config(void);
void EXTI_INFRAREDAVOID4_Config(void);

void EXTI9_5_IRQHandler(void);
#endif /* __BSP_EXTI_H */

