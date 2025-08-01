#include "csb.h"
#include "timer.h"
#include "delay.h"

void HCSR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 1. ����GPIOBʱ��
    RCC_AHB1PeriphClockCmd(GPIO_RCC, ENABLE);
    
    // 2. ����Trig����(PB4)Ϊ���
    GPIO_InitStruct.GPIO_Pin = TRIG_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_PORT, &GPIO_InitStruct);
    
    // 3. ����Echo����(PB5)Ϊ����
    GPIO_InitStruct.GPIO_Pin = ECHO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;  // ����ģʽ
    GPIO_Init(GPIO_PORT, &GPIO_InitStruct);
    
    // 4. ��ʼ״̬Trig�õ�
    GPIO_ResetBits(GPIO_PORT, TRIG_PIN);
    
    // 5. ��ʼ����ʱ��
    Timer_Init();
}

void HCSR04_Start(void)
{
    // ����10us���ϸߵ�ƽ����
    GPIO_SetBits(GPIO_PORT, TRIG_PIN);
    Delay_us(15);
    GPIO_ResetBits(GPIO_PORT, TRIG_PIN);
}

uint32_t HCSR04_GetDistance(void)
{
    Time = 0;
    HCSR04_Start();
    
    // �ȴ�Echo�źű��
    while(GPIO_ReadInputDataBit(GPIO_PORT, ECHO_PIN) == RESET);
    
    // ������ʱ�������ߵ�ƽ����ʱ��
    TIM4->CNT = 0;
    TIM_Cmd(TIM4, ENABLE);
    
    // �ȴ�Echo�źŽ���
    while(GPIO_ReadInputDataBit(GPIO_PORT, ECHO_PIN) == SET);
    TIM_Cmd(TIM4, DISABLE);  // ֹͣ����
    
    // ������� (cm): (ʱ�� * 0.034) / 2
    // Time��λΪ10us������ʵ��ʱ��ΪTime * 10us
    uint32_t duration_us = Time * 10;
    return (duration_us * 34) / 2000;  // �Ż�Ϊ�������㣺duration_us * 0.017
}

