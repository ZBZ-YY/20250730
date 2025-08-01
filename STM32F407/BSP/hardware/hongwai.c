#include "hongwai.h"
#include "stm32f4xx.h"

// 定义全局变量
volatile uint8_t avoidRightActive = 0;
volatile uint8_t avoidLeftActive = 0;
volatile uint8_t avoidForwardRightActive = 0;
volatile uint8_t avoidForwardLeftActive = 0;
volatile uint8_t avoidForwardActive = 0;
static void EXTI_INFRAREDAVOID_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 设置NVIC中断分组为2
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 配置EXTI9_5中断通道 (PD8和PD9共享此中断)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  // 修改中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// 红外避障传感器1配置 (PD8)
void EXTI_INFRAREDAVOID1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // 使能GPIOD时钟
    RCC_AHB1PeriphClockCmd(INFRAREDAVOID1_INT_GPIO_CLK, ENABLE);
    // 使能SYSCFG时钟（F4系列需要）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    // 配置PD8为浮空输入
    GPIO_InitStructure.GPIO_Pin = INFRAREDAVOID1_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // F4系列使用PuPd配置
    GPIO_Init(INFRAREDAVOID1_INT_GPIO_PORT, &GPIO_InitStructure);
    
    // 配置EXTI8映射到PD8
    SYSCFG_EXTILineConfig(INFRAREDAVOID1_INT_EXTI_PortSource, INFRAREDAVOID1_INT_EXTI_PinSource);
    
    // 配置EXTI8
    EXTI_InitStructure.EXTI_Line = INFRAREDAVOID1_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 仅下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置中断优先级
    EXTI_INFRAREDAVOID_NVIC_Config();
}

// 红外避障传感器2配置 (PD9)
void EXTI_INFRAREDAVOID2_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // 时钟已在第一个传感器配置中使能，此处无需重复
    
    // 配置PD9为浮空输入
    GPIO_InitStructure.GPIO_Pin = INFRAREDAVOID2_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(INFRAREDAVOID2_INT_GPIO_PORT, &GPIO_InitStructure);
    
    // 配置EXTI9映射到PD9
    SYSCFG_EXTILineConfig(INFRAREDAVOID2_INT_EXTI_PortSource, INFRAREDAVOID2_INT_EXTI_PinSource);
    
    // 配置EXTI9
    EXTI_InitStructure.EXTI_Line = INFRAREDAVOID2_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 仅下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 中断优先级已配置
}

// 红外避障传感器3配置 (PD7)
void EXTI_INFRAREDAVOID3_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // 时钟已在第一个传感器配置中使能，此处无需重复
    
    // 配置PD7为浮空输入
    GPIO_InitStructure.GPIO_Pin = INFRAREDAVOID3_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(INFRAREDAVOID3_INT_GPIO_PORT, &GPIO_InitStructure);
    
    // 配置EXTI7映射到PD7
    SYSCFG_EXTILineConfig(INFRAREDAVOID3_INT_EXTI_PortSource, INFRAREDAVOID3_INT_EXTI_PinSource);
    
    // 配置EXTI7
    EXTI_InitStructure.EXTI_Line = INFRAREDAVOID3_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 仅下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 中断优先级已配置
}
// 红外避障传感器4配置 (PD6)
void EXTI_INFRAREDAVOID4_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // 时钟已在第一个传感器配置中使能，此处无需重复
    
    // 配置PD6为浮空输入
    GPIO_InitStructure.GPIO_Pin = INFRAREDAVOID4_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(INFRAREDAVOID4_INT_GPIO_PORT, &GPIO_InitStructure);
    
    // 配置EXTI9映射到PD6
    SYSCFG_EXTILineConfig(INFRAREDAVOID4_INT_EXTI_PortSource, INFRAREDAVOID4_INT_EXTI_PinSource);
    
    // 配置EXTI6
    EXTI_InitStructure.EXTI_Line = INFRAREDAVOID4_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 仅下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 中断优先级已配置
}



