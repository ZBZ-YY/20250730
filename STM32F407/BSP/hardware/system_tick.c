#include "system_tick.h"
#include "stm32f4xx.h"

volatile uint32_t system_tick_count = 0;
volatile uint32_t ms_delay = 0;  // ������ʱ����

void SystemTick_Init(void) {
    // ����SysTickΪ1ms�ж�
    if (SysTick_Config(SystemCoreClock / 1000)) {
        // ��ʼ��ʧ�ܴ���
        while (1);
    }
    
    // ��ʼ��DWT������΢����ʱ��
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // ����SysTick�ж����ȼ�
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
}

uint32_t GetSystemTick(void) {
    return system_tick_count;
}

// SysTick�жϴ�����
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
    
    // ������������
    while (1) {
        uint32_t current = DWT->CYCCNT;
        uint32_t elapsed;
        
        if (current >= start) {
            elapsed = current - start;
        } else {
            // �������������
            elapsed = (0xFFFFFFFF - start) + current + 1;
        }
        
        if (elapsed >= cycles) {
            break;
        }
    }
}
