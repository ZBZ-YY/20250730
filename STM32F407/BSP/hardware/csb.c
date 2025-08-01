#include "csb.h"
#include "timer.h"
#include "delay.h"

void HCSR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 1. 启用GPIOB时钟
    RCC_AHB1PeriphClockCmd(GPIO_RCC, ENABLE);
    
    // 2. 配置Trig引脚(PB4)为输出
    GPIO_InitStruct.GPIO_Pin = TRIG_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_PORT, &GPIO_InitStruct);
    
    // 3. 配置Echo引脚(PB5)为输入
    GPIO_InitStruct.GPIO_Pin = ECHO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;  // 下拉模式
    GPIO_Init(GPIO_PORT, &GPIO_InitStruct);
    
    // 4. 初始状态Trig置低
    GPIO_ResetBits(GPIO_PORT, TRIG_PIN);
    
    // 5. 初始化定时器
    Timer_Init();
}

void HCSR04_Start(void)
{
    // 发送10us以上高电平脉冲
    GPIO_SetBits(GPIO_PORT, TRIG_PIN);
    Delay_us(15);
    GPIO_ResetBits(GPIO_PORT, TRIG_PIN);
}

uint32_t HCSR04_GetDistance(void)
{
    Time = 0;
    HCSR04_Start();
    
    // 等待Echo信号变高
    while(GPIO_ReadInputDataBit(GPIO_PORT, ECHO_PIN) == RESET);
    
    // 启动定时器测量高电平持续时间
    TIM4->CNT = 0;
    TIM_Cmd(TIM4, ENABLE);
    
    // 等待Echo信号结束
    while(GPIO_ReadInputDataBit(GPIO_PORT, ECHO_PIN) == SET);
    TIM_Cmd(TIM4, DISABLE);  // 停止计数
    
    // 计算距离 (cm): (时间 * 0.034) / 2
    // Time单位为10us，所以实际时间为Time * 10us
    uint32_t duration_us = Time * 10;
    return (duration_us * 34) / 2000;  // 优化为整数运算：duration_us * 0.017
}

