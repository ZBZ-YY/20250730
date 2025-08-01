#include "system_tick.h"
#include "stm32f4xx.h"

volatile uint32_t system_tick_count = 0;
volatile uint32_t ms_delay = 0;  // 用于延时函数

void SystemTick_Init(void) {
    // 配置SysTick为1ms中断
    if (SysTick_Config(SystemCoreClock / 1000)) {
        // 初始化失败处理
        while (1);
    }
    
    // 初始化DWT（用于微秒延时）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // 设置SysTick中断优先级
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
}

uint32_t GetSystemTick(void) {
    return system_tick_count;
}

// SysTick中断处理函数
void SysTick_Handler(void) {
    system_tick_count++;
    if (ms_delay > 0) {
        ms_delay--;
    }
}

void Delay_ms(uint32_t ms) {
    ms_delay = ms;
    while (ms_delay != 0);
}

void Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    
    // 处理计数器溢出
    while (1) {
        uint32_t current = DWT->CYCCNT;
        uint32_t elapsed;
        
        if (current >= start) {
            elapsed = current - start;
        } else {
            // 处理计数器回绕
            elapsed = (0xFFFFFFFF - start) + current + 1;
        }
        
        if (elapsed >= cycles) {
            break;
        }
    }
}
