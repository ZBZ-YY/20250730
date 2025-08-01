#include "stm32f4xx.h"
#include "delay.h"

// ȫ�ֱ���������� volatile��
volatile uint32_t ms_delay;  // ͳһ������Ϊ ms_delay

// ��ʼ��������������ã���
void Delay_Init(void)
{
    // ����SysTickΪ1ms�жϣ�168MHz��Ƶ��
    SysTick_Config(SystemCoreClock / 1000);
    
    // ��ʼ��DWT������΢����ʱ��
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// ���뼶��ʱ
void Delay_ms(uint32_t ms)
{
    ms_delay = ms;
    while(ms_delay != 0);
}

// ΢�뼶��ʱ��ʹ��DWT���ڼ�������
void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    // 168MHz�£�168 cycles = 1us
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    
    // ���������������
    if (cycles > (0xFFFFFFFF - start)) {
        // �ȴ�����������
        while(DWT->CYCCNT > start);
        start = 0;
    }
    
    // �ȴ��㹻������
    while((DWT->CYCCNT - start) < cycles);
}


