#include "stm32f4xx.h"                  // Device header
#include "PWM.h"


void PWM_Motor_Init(void)
{
    // ���������б�����ȷ�����κο�ִ�����֮ǰ��
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // ʹ��ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // ����GPIOΪ���ù��ܣ�PA0-PA3��ӦTIM2 CH1-CH4��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // ���ø��ù���ӳ�䵽TIM2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);  // CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);  // CH2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);  // CH3
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);  // CH4
    
    // ����TIM2ʱ����Ԫ��Ŀ��Ƶ��10kHz��
    TIM_InternalClockConfig(TIM2);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;      
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;    
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    // ����TIM2����Ƚϵ�Ԫ����ʼռ�ձ�50%��
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  
    
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);  // ͨ��1
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);  // ͨ��2
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);  // ͨ��3
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);  // ͨ��4
   
    
    // ����TIM2
    TIM_Cmd(TIM2, ENABLE);
}

void PWM_Servo_Init(void)
{
    // 1. ʹ��ʱ�� (F407ʱ������F1��ͬ)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  // TIM3��APB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // GPIOA��AHB1

    // 2. ����GPIO���ù���
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // PA6(TIM3_CH1), PA7(TIM3_CH2)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;          // ����ģʽ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    // F4֧��100MHz
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;        // �������
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;          // ����
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // �������Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);  // PA6����ΪTIM3_CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);  // PA7����ΪTIM3_CH2

    // 3. ���ö�ʱ��ʱ����Ԫ (F407��Ƶ168MHz)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;      // 20ms���� (50Hz PWM)
    TIM_TimeBaseInitStruct.TIM_Prescaler = 84 - 1;      
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    // 4. ����PWM���ͨ��
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);  // ��ʼ��Ĭ��ֵ
    
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_Pulse = 1500;    // ��ʼ����1.5ms (��λ)

    // Ӧ�����õ�����ͨ��
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);  // ͨ��1
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);  // ͨ��2

    // 5. ����Ԥװ�ؼĴ���
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // 6. ������ʱ��
    TIM_Cmd(TIM3, ENABLE);
}

/**
 * ����TIM2ͨ��1��PWMռ�ձ�
 */
void PWM_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM2, Compare);
}

/**
 * ����TIM2ͨ��2��PWMռ�ձ�
 */
void PWM_SetCompare2(uint16_t Compare)
{
    TIM_SetCompare2(TIM2, Compare);
}


/**
 * ����TIM2ͨ��3��PWMռ�ձ�
 */
void PWM_SetCompare3(uint16_t Compare)
{
    TIM_SetCompare3(TIM2, Compare);
}

/**
 * ����TIM2ͨ��4��PWMռ�ձ�
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
