#include "timer.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

volatile uint32_t Time = 0;

void Timer_Init(void)
{
    // 1. ����TIM4ʱ�� (APB1����)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 2. ����ʱ�� (84MHzʱ�ӣ�10us�ж�����)
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_InitStruct.TIM_Prescaler = 83;         // 84MHz/(83+1) = 1MHz �� 1us
    TIM_InitStruct.TIM_Period = 9;             // 10us�ж�һ�� (10-1)
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_InitStruct);
    
    // 3. �����־λ�������ж�
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    // 4. ����NVIC
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 5. ���ö�ʱ��
    TIM_Cmd(TIM4, ENABLE);
}

// TIM4�жϴ�����
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        // ���Echo����(PB5)Ϊ�ߵ�ƽ������ʱ�����
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == SET)
        {
            Time++;
        }
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}


