#include "hongwai.h"
#include "stm32f4xx.h"

// ����ȫ�ֱ���
volatile uint8_t avoidRightActive = 0;
volatile uint8_t avoidLeftActive = 0;
volatile uint8_t avoidForwardRightActive = 0;
volatile uint8_t avoidForwardLeftActive = 0;
volatile uint8_t avoidForwardActive = 0;
static void EXTI_INFRAREDAVOID_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // ����NVIC�жϷ���Ϊ2
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // ����EXTI9_5�ж�ͨ�� (PD8��PD9������ж�)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  // �޸��ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// ������ϴ�����1���� (PD8)
void EXTI_INFRAREDAVOID1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // ʹ��GPIODʱ��
    RCC_AHB1PeriphClockCmd(INFRAREDAVOID1_INT_GPIO_CLK, ENABLE);
    // ʹ��SYSCFGʱ�ӣ�F4ϵ����Ҫ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    // ����PD8Ϊ��������
    GPIO_InitStructure.GPIO_Pin = INFRAREDAVOID1_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // F4ϵ��ʹ��PuPd����
    GPIO_Init(INFRAREDAVOID1_INT_GPIO_PORT, &GPIO_InitStructure);
    
    // ����EXTI8ӳ�䵽PD8
    SYSCFG_EXTILineConfig(INFRAREDAVOID1_INT_EXTI_PortSource, INFRAREDAVOID1_INT_EXTI_PinSource);
    
    // ����EXTI8
    EXTI_InitStructure.EXTI_Line = INFRAREDAVOID1_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // ���½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // �����ж����ȼ�
    EXTI_INFRAREDAVOID_NVIC_Config();
}

// ������ϴ�����2���� (PD9)
void EXTI_INFRAREDAVOID2_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // ʱ�����ڵ�һ��������������ʹ�ܣ��˴������ظ�
    
    // ����PD9Ϊ��������
    GPIO_InitStructure.GPIO_Pin = INFRAREDAVOID2_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(INFRAREDAVOID2_INT_GPIO_PORT, &GPIO_InitStructure);
    
    // ����EXTI9ӳ�䵽PD9
    SYSCFG_EXTILineConfig(INFRAREDAVOID2_INT_EXTI_PortSource, INFRAREDAVOID2_INT_EXTI_PinSource);
    
    // ����EXTI9
    EXTI_InitStructure.EXTI_Line = INFRAREDAVOID2_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // ���½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // �ж����ȼ�������
}

// ������ϴ�����3���� (PD7)
void EXTI_INFRAREDAVOID3_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // ʱ�����ڵ�һ��������������ʹ�ܣ��˴������ظ�
    
    // ����PD7Ϊ��������
    GPIO_InitStructure.GPIO_Pin = INFRAREDAVOID3_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(INFRAREDAVOID3_INT_GPIO_PORT, &GPIO_InitStructure);
    
    // ����EXTI7ӳ�䵽PD7
    SYSCFG_EXTILineConfig(INFRAREDAVOID3_INT_EXTI_PortSource, INFRAREDAVOID3_INT_EXTI_PinSource);
    
    // ����EXTI7
    EXTI_InitStructure.EXTI_Line = INFRAREDAVOID3_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // ���½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // �ж����ȼ�������
}
// ������ϴ�����4���� (PD6)
void EXTI_INFRAREDAVOID4_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    // ʱ�����ڵ�һ��������������ʹ�ܣ��˴������ظ�
    
    // ����PD6Ϊ��������
    GPIO_InitStructure.GPIO_Pin = INFRAREDAVOID4_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(INFRAREDAVOID4_INT_GPIO_PORT, &GPIO_InitStructure);
    
    // ����EXTI9ӳ�䵽PD6
    SYSCFG_EXTILineConfig(INFRAREDAVOID4_INT_EXTI_PortSource, INFRAREDAVOID4_INT_EXTI_PinSource);
    
    // ����EXTI6
    EXTI_InitStructure.EXTI_Line = INFRAREDAVOID4_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // ���½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // �ж����ȼ�������
}



