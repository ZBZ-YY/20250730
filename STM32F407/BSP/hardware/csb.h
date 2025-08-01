#ifndef __CSB_H
#define __CSB_H

#include "stm32f4xx.h"

// Òý½Å¶¨Òå - PB6(Trig), PB7(Echo)
#define TRIG_PIN     GPIO_Pin_6
#define ECHO_PIN     GPIO_Pin_7
#define GPIO_PORT    GPIOB
#define GPIO_RCC     RCC_AHB1Periph_GPIOB

void HCSR04_Init(void);
void HCSR04_Start(void);
uint32_t HCSR04_GetDistance(void);

#endif

