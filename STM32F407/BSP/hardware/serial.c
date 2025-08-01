#include "stm32f4xx.h"
#include "Serial.h"
#include <stdarg.h>

// 定义状态机状态
typedef enum {
    STATE_WAIT_HEADER,
    STATE_RECV_SERVO_X,
    STATE_RECV_SERVO_Y,
    STATE_RECV_DISTANCE,
    STATE_WAIT_FOOTER
} UART_State;

// 全局变量声明
volatile uint8_t Serial_RxFlag = 0;  // 完整数据接收标志
uint8_t Serial_RxPacket[3];          // 接收数据缓冲区

void Serial_Init(void) {
    // 1. 使能时钟 (F407使用AHB1和APB2)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
    // 2. 配置GPIO复用功能
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // PA9 (USART1_TX) 复用推挽输出
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // PA10 (USART1_RX) 复用输入
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // 浮空输入(无上拉/下拉)
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 设置引脚复用映射 (USART1对应AF7)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    
    // 3. 配置USART
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);
    
    // 4. 配置接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    // 5. 配置NVIC
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 6. 使能串口
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void) {
    static UART_State state = STATE_WAIT_HEADER;  // 状态机状态
    uint8_t RxByte;
    
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        RxByte = USART_ReceiveData(USART1);
        
        switch (state) {  // 使用局部状态变量
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
                    Serial_RxFlag = 1; // 接收完成
                }
                // 无论包尾是否正确，都回到等待包头状态
                state = STATE_WAIT_HEADER;
                break;
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
