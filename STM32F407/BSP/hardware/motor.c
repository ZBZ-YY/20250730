#include "stm32f4xx.h"
#include "motor.h"
#include "PWM.h"
#include <math.h>  // 添加数学函数支持
// 差速转向参数定义
#define BASE_SPEED 20     // 基础速度
//#define DIFFERENTIAL_GAIN 1.2f // 差速增益系数（决定内外轮速度差程度）（高人的）
#define DIFFERENTIAL_GAIN 1.3f // 差速增益系数（决定内外轮速度差程度）（这个场地的）
#define MIN_DIFF 15      // 最小速度差（确保至少有一定差速）

void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 使能GPIOB和GPIOE时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);
   
    // 控制电机方向的 GPIO 用普通输出模式
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    

    // 配置控制电机 IN1~IN8 的引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | 
                                  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    // 初始化电机控制所需的 PWM（假设用于调速等，保持你原有的调用逻辑 ）
    PWM_Motor_Init();
}



// 停止函数（不需要速度参数）
void Car_Stop(void) {
    // 所有IN口置为低电平
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // 设置所有PWM输出为0
    PWM_SetCompare1(0);
    PWM_SetCompare2(0);
    PWM_SetCompare3(0);
    PWM_SetCompare4(0);
}

// 前进函数（可设置速度）
void Car_GoForward(uint8_t speed) {
    // 右前轮 (PE14/PE15)
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    // 左后轮 (PB10/PB11)
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    // 右后轮 (PB12/PB13)
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    // 左前轮 (PB14/PB15)
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    /*高人的，速度100
    // 设置所有电机PWM速度
    PWM_SetCompare1(speed-2);  // PA0 右前轮motorC
    PWM_SetCompare2(speed-2);  // PA1 左后轮motorA
    PWM_SetCompare3(speed-2);  // PA2 右后轮motorB
    PWM_SetCompare4(speed);  // PA3 左前轮motorD
		// 设置所有电机PWM速度*/
		//下面的是速度120
    PWM_SetCompare1(speed-2);  // PA0 右前轮motorC
    PWM_SetCompare2(speed-2);  // PA1 左后轮motorA
    PWM_SetCompare3(speed-2);  // PA2 右后轮motorB
    PWM_SetCompare4(speed);  // PA3 左前轮motorD
}


void Car_TurnSmoothLeft(uint8_t angle) {
    // 计算转向角度比例（0到1，0表示直行，1表示最大转向）
    float turnRatio = fabsf(angle - 90.0f) / 90.0f;
    
    // 计算内轮和外轮的速度差
    float speedDiff = (float)BASE_SPEED * DIFFERENTIAL_GAIN * turnRatio;
    speedDiff = (speedDiff < MIN_DIFF) ? MIN_DIFF : speedDiff;
    
    // 计算内外轮实际速度
    uint8_t innerSpeed = (uint8_t)fmaxf(0, BASE_SPEED - speedDiff);
    uint8_t outerSpeed = (uint8_t)fminf(255, BASE_SPEED + speedDiff);
    
    // 左转时：右轮是外轮（更快），左轮是内轮（更慢）
    // 右前轮前进 (PE14/PE15) - 外轮最快
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    // 左后轮前进 (PB10/PB11) - 内轮最慢
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    // 右后轮前进 (PB12/PB13) - 外轮
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    // 左前轮前进 (PB14/PB15) - 内轮
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // 设置PWM速度 - 内侧轮更慢，外侧轮更快
    PWM_SetCompare1(outerSpeed);  // 右前轮（外轮最快）
    PWM_SetCompare2(innerSpeed);  // 左后轮（内轮最慢）
    PWM_SetCompare3(outerSpeed);  // 右后轮（外轮）
    PWM_SetCompare4(innerSpeed);  // 左前轮（内轮）
}

void Car_TurnSmoothRight(uint8_t angle) {
    // 计算转向角度比例
    float turnRatio = fabsf(angle - 90.0f) / 90.0f;
    
    // 计算内轮和外轮的速度差
    float speedDiff = (float)BASE_SPEED * DIFFERENTIAL_GAIN * turnRatio;
    speedDiff = (speedDiff < MIN_DIFF) ? MIN_DIFF : speedDiff;
    
    // 计算内外轮实际速度
    uint8_t innerSpeed = (uint8_t)fmaxf(0, BASE_SPEED - speedDiff);
    uint8_t outerSpeed = (uint8_t)fminf(255, BASE_SPEED + speedDiff);
    
    // 右转时：左轮是外轮（更快），右轮是内轮（更慢）
    // 右前轮前进 (PE14/PE15) - 内轮
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    // 左后轮前进 (PB10/PB11) - 外轮
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    // 右后轮前进 (PB12/PB13) - 内轮最慢
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    // 左前轮前进 (PB14/PB15) - 外轮最快
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // 设置PWM速度 - 内侧轮更慢，外侧轮更快
    PWM_SetCompare1(innerSpeed);  // 右前轮（内轮）
    PWM_SetCompare2(outerSpeed);  // 左后轮（外轮）
    PWM_SetCompare3(innerSpeed);  // 右后轮（内轮最慢）
    PWM_SetCompare4(outerSpeed);  // 左前轮（外轮最快）
}

// 通用转向函数（集成到追踪控制中）
void Car_Steer(uint8_t baseSpeed, float steeringAngle) {
    // 计算转向比例（-1.0完全左转，0直行，+1.0完全右转）
    float steerRatio = (steeringAngle - 90.0f) / 90.0f;
    
    // 计算速度差（转向越急，差速越大）
    float speedDiff = fabsf(steerRatio) * baseSpeed * DIFFERENTIAL_GAIN;
    speedDiff = fmaxf(speedDiff, MIN_DIFF);
    
    // 设置所有轮子前进方向
    GPIO_SetBits(GPIOE, GPIO_Pin_14); // 右前
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_11); // 左后
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_13); // 右后
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_14); // 左前
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // 根据转向方向设置差速
    if(steerRatio < 0) { // 左转
        PWM_SetCompare1(baseSpeed + speedDiff); // 右前（外轮）更快
        PWM_SetCompare2(baseSpeed - speedDiff); // 左后（内轮）更慢
        PWM_SetCompare3(baseSpeed + speedDiff); // 右后（外轮）更快
        PWM_SetCompare4(baseSpeed - speedDiff); // 左前（内轮）更慢
    } 
    else if(steerRatio > 0) { // 右转
        PWM_SetCompare1(baseSpeed - speedDiff); // 右前（内轮）更慢
        PWM_SetCompare2(baseSpeed + speedDiff); // 左后（外轮）更快
        PWM_SetCompare3(baseSpeed - speedDiff); // 右后（内轮）更慢
        PWM_SetCompare4(baseSpeed + speedDiff); // 左前（外轮）更快
    }
    else { // 直行
        PWM_SetCompare1(baseSpeed);
        PWM_SetCompare2(baseSpeed);
        PWM_SetCompare3(baseSpeed);
        PWM_SetCompare4(baseSpeed);
    }
}
// 右转函数（可设置速度）
void Car_TurnRight(uint8_t speed) {
    // 右前轮后退 (PE14/PE15)
    GPIO_SetBits(GPIOE, GPIO_Pin_15);
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    
    // 左后轮前进 (PB10/PB11)
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    // 右后轮后退 (PB12/PB13)
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    
    // 左前轮前进 (PB14/PB15)
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // 设置PWM速度
    PWM_SetCompare1(speed-2);  // PA0 右前轮motorC
    PWM_SetCompare2(speed-2);  // PA1 左后轮motorA
    PWM_SetCompare3(speed-2);  // PA2 右后轮motorB
    PWM_SetCompare4(speed);  // PA3 左前轮motorD
}

// 左转函数（可设置速度）
void Car_TurnLeft(uint8_t speed) {
    // 右前轮前进 (PE14/PE15)
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    // 左后轮后退 (PB10/PB11)
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    
    // 右后轮前进 (PB12/PB13)
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    // 左前轮后退 (PB14/PB15)
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    
    // 设置PWM速度
    PWM_SetCompare1(speed-2);  // PA0 右前轮motorC
    PWM_SetCompare2(speed-2);  // PA1 左后轮motorA
    PWM_SetCompare3(speed-2);  // PA2 右后轮motorB
    PWM_SetCompare4(speed);  // PA3 左前轮motorD
}

void Car_Back(uint8_t speed) {
    // 右前轮 (PE14/PE15)
    GPIO_SetBits(GPIOE, GPIO_Pin_15);
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    
    // 左后轮 (PB10/PB11)
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    
    // 右后轮 (PB12/PB13)
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    
    // 左前轮 (PB14/PB15)
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    
    // 设置所有电机PWM速度
    PWM_SetCompare1(speed);  // PA0 右前轮
    PWM_SetCompare2(speed);  // PA1 左后轮
    PWM_SetCompare3(speed);  // PA2 右后轮
    PWM_SetCompare4(speed);  // PA3 左前轮
}

