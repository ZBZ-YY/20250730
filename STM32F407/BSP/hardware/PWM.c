#include "stm32f4xx.h"                  // Device header
#include "PWM.h"


void PWM_Motor_Init(void)
{
    // 先声明所有变量（确保在任何可执行语句之前）
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 配置GPIO为复用功能（PA0-PA3对应TIM2 CH1-CH4）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置复用功能映射到TIM2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);  // CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);  // CH2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);  // CH3
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);  // CH4
    
    // 配置TIM2时基单元（目标频率10kHz）
    TIM_InternalClockConfig(TIM2);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;      
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;    
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    // 配置TIM2输出比较单元（初始占空比50%）
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  
    
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);  // 通道1
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);  // 通道2
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);  // 通道3
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);  // 通道4
   
    
    // 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
}

void PWM_Servo_Init(void)
{
    // 1. 使能时钟 (F407时钟树与F1不同)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  // TIM3在APB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // GPIOA在AHB1

    // 2. 配置GPIO复用功能
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // PA6(TIM3_CH1), PA7(TIM3_CH2)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;          // 复用模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    // F4支持100MHz
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;        // 推挽输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;          // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 设置引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);  // PA6复用为TIM3_CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);  // PA7复用为TIM3_CH2

    // 3. 配置定时器时基单元 (F407主频168MHz)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;      // 20ms周期 (50Hz PWM)
    TIM_TimeBaseInitStruct.TIM_Prescaler = 84 - 1;      
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    // 4. 配置PWM输出通道
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);  // 初始化默认值
    
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_Pulse = 1500;    // 初始脉宽1.5ms (中位)

    // 应用配置到两个通道
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);  // 通道1
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);  // 通道2

    // 5. 启用预装载寄存器
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // 6. 启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

/**
 * 设置TIM2通道1的PWM占空比
 */
void PWM_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM2, Compare);
}

/**
 * 设置TIM2通道2的PWM占空比
 */
void PWM_SetCompare2(uint16_t Compare)
{
    TIM_SetCompare2(TIM2, Compare);
}


/**
 * 设置TIM2通道3的PWM占空比
 */
void PWM_SetCompare3(uint16_t Compare)
{
    TIM_SetCompare3(TIM2, Compare);
}

/**
 * 设置TIM2通道4的PWM占空比
 */
void PWM_SetCompare4(uint16_t Compare)
{
    TIM_SetCompare4(TIM2, Compare);
}






void PWM_SetCompare5(uint16_t Compare)
{
    TIM_SetCompare1(TIM3, Compare);
}

void PWM_SetCompare6(uint16_t Compare)
{
    TIM_SetCompare2(TIM3, Compare);
}
