#include "stm32f4xx.h"
#include "Serial.h"
#include <stdarg.h>

// ����״̬��״̬
typedef enum {
    STATE_WAIT_HEADER,
    STATE_RECV_SERVO_X,
    STATE_RECV_SERVO_Y,
    STATE_RECV_DISTANCE,
    STATE_WAIT_FOOTER
} UART_State;

// ȫ�ֱ�������
volatile uint8_t Serial_RxFlag = 0;  // �������ݽ��ձ�־
uint8_t Serial_RxPacket[3];          // �������ݻ�����

void Serial_Init(void) {
    // 1. ʹ��ʱ�� (F407ʹ��AHB1��APB2)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
    // 2. ����GPIO���ù���
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // PA9 (USART1_TX) �����������
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // PA10 (USART1_RX) ��������
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��������(������/����)
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // �������Ÿ���ӳ�� (USART1��ӦAF7)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    
    // 3. ����USART
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);
    
    // 4. ���ý����ж�
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    // 5. ����NVIC
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 6. ʹ�ܴ���
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void) {
    static UART_State state = STATE_WAIT_HEADER;  // ״̬��״̬
    uint8_t RxByte;
    
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        RxByte = USART_ReceiveData(USART1);
        
        switch (state) {  // ʹ�þֲ�״̬����
            case STATE_WAIT_HEADER:
                if (RxByte == 0xFF) {
                    state = STATE_RECV_SERVO_X;
                }
                break;
                
            case STATE_RECV_SERVO_X:
                Serial_RxPacket[0] = RxByte; 
                state = STATE_RECV_SERVO_Y;
                break;
                
            case STATE_RECV_SERVO_Y:
                Serial_RxPacket[1] = RxByte; 
                state = STATE_RECV_DISTANCE;
                break;
                
            case STATE_RECV_DISTANCE:
                Serial_RxPacket[2] = RxByte; 
                state = STATE_WAIT_FOOTER;
                break;
                
            case STATE_WAIT_FOOTER:
                if (RxByte == 0xFE) {
                    Serial_RxFlag = 1; // �������
                }
                // ���۰�β�Ƿ���ȷ�����ص��ȴ���ͷ״̬
                state = STATE_WAIT_HEADER;
                break;
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
