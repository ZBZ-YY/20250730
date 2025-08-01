#include "stm32f4xx.h"                  // Device header
#include "PWM.h"
#include <math.h>
static float currentAngle1 = 0.0f;  // 舵机1当前角度
static float currentAngle2 = 0.0f;  // 舵机2当前角度

void Servo_Init(void)
{
	PWM_Servo_Init();
}

void Servo1_SetAngle(float Angle)
{
	uint32_t pulseWidth = (uint32_t)(500.0f + Angle * (2000.0f / 270.0f));
	PWM_SetCompare5(pulseWidth);
	currentAngle1 = Angle;  // 保存当前角度
}

void Servo2_SetAngle(float Angle)
{
	uint32_t pulseWidth = (uint32_t)(500.0f + Angle * (2000.0f / 180.0f));
	PWM_SetCompare6(pulseWidth);
	currentAngle2 = Angle;  // 保存当前角度
}

float Servo1_GetAngle(void)
{
    return currentAngle1;
}

float Servo2_GetAngle(void)
{
    return currentAngle2;
}
