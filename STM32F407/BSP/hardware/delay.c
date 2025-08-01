#include "stm32f4xx.h"
#include "delay.h"

// 全局变量（必须加 volatile）
volatile uint32_t ms_delay;  // 统一变量名为 ms_delay

// 初始化函数（必须调用！）
void Delay_Init(void)
{
    // 配置SysTick为1ms中断（168MHz主频）
    SysTick_Config(SystemCoreClock / 1000);
    
    // 初始化DWT（用于微秒延时）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 毫秒级延时
void Delay_ms(uint32_t ms)
{
    ms_delay = ms;
    while(ms_delay != 0);
}

// 微秒级延时（使用DWT周期计数器）
void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    // 168MHz下：168 cycles = 1us
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    
    // 处理计数器溢出情况
    if (cycles > (0xFFFFFFFF - start)) {
        // 等待计数器回绕
        while(DWT->CYCCNT > start);
        start = 0;
    }
    
    // 等待足够周期数
    while((DWT->CYCCNT - start) < cycles);
}


