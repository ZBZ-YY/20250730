#ifndef __PWM_H
#define __PWM_H

#include "stm32f4xx.h"

/* �������� */
void PWM_Motor_Init(void);
void PWM_Servo_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void PWM_SetCompare3(uint16_t Compare);
void PWM_SetCompare4(uint16_t Compare);
void PWM_SetCompare5(uint16_t Compare);
void PWM_SetCompare6(uint16_t Compare);
#endif /* __PWM_H */

