/***********************************************************************************************************************************
 ** �������д��  ħŮ�������Ŷ�
 ** �����汾��  2024-07-08-01
 ** ����    ����  https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** ���ļ����ơ�  bsp_UART.c
 **
 ** ���ļ����ܡ�  ��UART��GPIO���á�ͨ��Э�����á��ж����ã������ܺ���ʵ��
 **
 ** ������ƽ̨��  STM32F407 + keil5 + HAL��/��׼��
 **
 ** ��������]   20240708-1
 **         
 ** ���ر�˵����  1��Ϊʲô����ļ��У�ͬʱ�б�׼�⡢HAL��Ĵ��룿
 **                  ��Ϊ�˷�����ֲ��������Ԥ���봦���������ڱ�׼�⡢HAL��Ĺ�����ֲ��
 **               2��ΪʲôUART��ʼ�����жϣ��üĴ����������������ø��пɶ��Ե�HAL�⣿
 **                  ��ΪCubeMX���ù���ʱ���������UART�������ã��Ҵ�ֻ������Ҫ���ļ��������н�û��UART��HAL֧���ļ��ġ�
 **                  ����, �ж����üĴ�����������HAL������ط�װ��ȣ����Եظ���Ч��
************************************************************************************************************************************/
#include "bsp_UART.h"             // ͷ�ļ�




/*****************************************************************************
 ** �������ر���
****************************************************************************/
typedef struct
{
    uint16_t  usRxNum;            // ��һ֡���ݣ����յ����ٸ��ֽ�����
    uint8_t  *puRxData;           // ��һ֡���ݣ����ݻ���; ��ŵ��ǿ����жϺ󣬴���ʱ���ջ��渴�ƹ������������ݣ����ǽ��չ����еĲ���������;

    uint8_t  *puTxFiFoData;       // ���ͻ����������ζ���; Ϊ�˷�������Ķ���û�з�װ�ɶ��к���
    uint16_t  usTxFiFoData ;      // ���λ������Ķ�ͷ
    uint16_t  usTxFiFoTail ;      // ���λ������Ķ�β
} xUSATR_TypeDef;





/******************************************************************************
 * ��  ���� delay_ms
 * ��  �ܣ� ms ��ʱ����
 * ��  ע�� 1��ϵͳʱ��168MHz
 *          2���򹴣�Options/ c++ / One ELF Section per Function
            3�������Ż�����Level 3(-O3)
 * ��  ���� uint32_t  ms  ����ֵ
 * ����ֵ�� ��
 ******************************************************************************/
static volatile uint32_t ulTimesMS;    // ʹ��volatile��������ֹ�������������Ż�
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;                   // �����ⲿ��������ֹ��ѭ�����������Ż���
}





//////////////////////////////////////////////////////////////   UART-1   ///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART1_EN

static xUSATR_TypeDef  xUART1 = { 0 };                      // ���� UART1 ���շ��ṹ��
static uint8_t uaUART1RxData[UART1_RX_BUF_SIZE];            // ���� UART1 �Ľ��ջ���
static uint8_t uaUART1TxFiFoData[UART1_TX_BUF_SIZE];        // ���� UART1 �ķ��ͻ���

/******************************************************************************
 * ��  ���� UART1_Init
 * ��  �ܣ� ��ʼ��USART1��ͨ�����š�Э��������ж����ȼ�
 *          ���ţ�TX-PA10��RX-PA11
 *          Э�飺������-None-8-1
 *          ���ͣ������ж�
 *          ���գ�����+�����ж�
 *
 * ��  ���� uint32_t  ulBaudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/
void UART1_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    // ʹ�� ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);           // ʹ�� GPIOA  ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);          // ʹ�� USART1 ʱ��              
    // ���� ���ŵĸ��ù���                                       
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);       // ����PA9���ù��ܣ� USART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);      // ����PA10���ù��ܣ�USART1
    // ���� USART                                                  
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);          // ʹ������
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);         // ȡ������
    // ���� TX����                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO ��ʼ���ṹ��
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;                     // ���ű��
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // ���ŷ���: ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // ���ģʽ������
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // ������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // ����ٶȣ�50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ����R X����                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                    // ���ű��
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� UART                                                   
    USART_InitTypeDef USART_InitStructure;                          // ����USART��ʼ���ṹ��  
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // ���ò�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �����ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ����һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(USART1, &USART_InitStructure);                       // ��ʼ��USART
    // ���� �ж�
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                  // ���� �����ж�
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);                  // ���� �����ж�    
    // �����ж����ȼ�
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // �ж����ȼ����ýṹ��
    NVIC_InitStructure .NVIC_IRQChannel = USART1_IRQn;              // ָ���ж�ͨ��
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // ������ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // ������Ӧ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);                                 // ��ʼ��NVIC
    // ������ɣ�����USART
    USART_Cmd(USART1, ENABLE);                                      // ʹ��USART1
#endif

#ifdef USE_HAL_DRIVER                                               // HAL�� ����
    // ʹ�� ʱ��                                                
    __HAL_RCC_GPIOA_CLK_ENABLE();                                   // ʹ��GPIOA
    __HAL_RCC_USART1_CLK_ENABLE();                                  // ʹ��USART1
    // ���� USART                                                 
    __HAL_RCC_USART1_FORCE_RESET();                                 // ʹ������
    __HAL_RCC_USART1_RELEASE_RESET();                               // ȡ������
    // ���� ����                                        
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // ������ʼ��Ҫ�õ��Ľṹ��
    GPIO_InitStruct.Pin   = GPIO_PIN_9 | GPIO_PIN_10;               // ���� TX-PA9��RX-PA10
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // ����ģʽ
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // ������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // ��������
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;                    // ���Ÿ��ù���
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                         // ��ʼ�����Ź���ģʽ
    // ���㲨���ʲ���                                              
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // ����ϵͳ����Ƶ��ȫ��ֵ; ����SystemCoreClock( )���ڱ�׼�⡢HAL��ͨ��
    temp = (float)(SystemCoreClock / 2) / (ulBaudrate * 16);        // �����ʹ�ʽ����; USART1������APB2, ʱ��Ϊϵͳʱ�ӵ�2��Ƶ; ȫ�ֱ���SystemCoreClock���ڱ�׼�⡢HAL��ͨ��;
    mantissa = temp;				                                // ��������
    fraction = (temp - mantissa) * 16;                              // С������
    USART1 -> BRR  = mantissa << 4 | fraction;                      // ���ò�����
    // ���� USART                                            
    USART1 -> CR1  =   0;                                           // ��0
    USART1 -> CR1 |=   0x01 << 2;                                   // ����ʹ��[02]: 0=ʧ�ܡ�1=ʹ��
    USART1 -> CR1 |=   0x01 << 3;                                   // ����ʹ��[03]��0=ʧ�ܡ�1=ʹ��
    USART1 -> CR1 |=   0x00 << 9;                                   // ��żУ��[09]��0=żEven��1=��Odd;  ע�⣺ʹ����żУ�飬��������Ҫ��1
    USART1 -> CR1 |=   0x00 << 10;                                  // У��λ  [10]��0=���á�1=ʹ��;     ע�⣬ʹ����żУ�飬��λҪ��1 (������Ч�����ݴ���)
    USART1 -> CR1 |=   0x00 << 12;                                  // ����λ  [12]��0=8λ�� 1=9λ;      ע�⣺ʹ����żУ�飬��λҪ��1 (�������ݻᷢ������)
    USART1 -> CR2  =   0;                                           // ������0
    USART1 -> CR2 &= ~(0x03 << 12);                                 // ֹͣλ[13:12]��00b=1��ֹͣλ��01b=0.5��ֹͣλ��10b=2��ֹͣλ��11b=1.5��ֹͣλ
    USART1 -> CR3  =   0;                                           // ������0
    USART1 -> CR3 &= ~(0x01 << 6);                                  // DMA����[6]: 0=��ֹ��1=ʹ��
    USART1 -> CR3 &= ~(0x01 << 7);                                  // DMA����[7]: 0=��ֹ��1=ʹ��
    // ���� �ж�                                                 
    USART1 -> CR1 &= ~(0x01 << 7);                                  // �رշ����ж�
    USART1 -> CR1 |=   0x01 << 5;                                   // ʹ�ܽ����ж�: ���ջ������ǿ�
    USART1 -> CR1 |=   0x01 << 4;                                   // ʹ�ܿ����жϣ�����1�ֽ�ʱ��û�յ�������
    USART1 -> SR   = ~(0x00F0);                                     // �����ж�
    // ���� �ж����ȼ�                                              
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);                        // ����ָ���жϵ���Ӧ���ȼ�;  �������ж������š���ռ���������ȼ�
    HAL_NVIC_EnableIRQ(USART1_IRQn);                                // ʹ�ܡ�����ָ�����ж�
    // ������ɣ�����USART
    USART1 -> CR1 |=   0x01 << 13;                                  // ʹ��UART��ʼ����
#endif

    // ����������                                                   
    xUART1.puRxData = uaUART1RxData;                                // �������ջ������ĵ�ַ
    xUART1.puTxFiFoData = uaUART1TxFiFoData;                        // �������ͻ������ĵ�ַ   
    
    // �����ʾ
    printf("\r\r\r===========  STM32F407VE ���� ��ʼ������ ===========\r");                   // �������������
    SystemCoreClockUpdate();                                                                  // ����һ��ϵͳ����Ƶ�ʱ���
    printf("ϵͳʱ��Ƶ��             %d MHz\r", SystemCoreClock / 1000000);                   // �������������
    printf("UART1 ��ʼ������         %d-None-8-1; ����ɳ�ʼ�����á��շ�����\r", ulBaudrate); // �������������
}

/******************************************************************************
 * ��  ���� USART1_IRQHandler
 * ��  �ܣ� USART1�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 * ��  ע�� ���������������ж��¼�ʱ����Ӳ�����á�
 *          ���ʹ�ñ��ļ����룬�ڹ����ļ��������ط���Ҫע��ͬ�������������ͻ��
******************************************************************************/
void USART1_IRQHandler(void)
{
    static uint16_t cnt = 0;                                             // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  rxTemp[UART1_RX_BUF_SIZE];                           // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽�ⲿ���棺xUARTx.puRxData[ ]
                                                                         
    // �����жϣ����ڰѻ��λ�������ݣ����ֽڷ���                        
    if ((USART1->SR & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))  // ��鷢�ͼĴ������ж�ʹ�ܣ��ҷ��ͼĴ���Ϊ��; TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                                                                    
        USART1->DR = xUART1.puTxFiFoData[xUART1.usTxFiFoTail++];         // ��FIFO������ȡ��һ�����ݣ�����USART�ķ��ͼĴ���(Ӳ�����Զ�����)��Ȼ��FIFO��βָ�������ָ����һ��Ҫ���͵�����
        if (xUART1.usTxFiFoTail == UART1_TX_BUF_SIZE)                    // ���FIFOβָ���Ƿ񵽴���FIFO���е�ĩβ
            xUART1.usTxFiFoTail = 0;                                     // ��βָ������Ϊ0��ʵ�ֻ��ζ��еĹ���
        if (xUART1.usTxFiFoTail == xUART1.usTxFiFoData)                  // ���FIFOβָ���Ƿ�׷����ͷָ�룬�����������Ƿ��ѷ������
            USART1->CR1 &= ~USART_CR1_TXEIE;                             // �رշ��ͼĴ������жϣ���ֹ�жϷ�����򱻲���Ҫ�ص���
        return;                                                          
    }                                                                    
                                                                         
    // �����жϣ���������ֽڽ��գ���ŵ���ʱ����                        
    if (USART1->SR & USART_SR_RXNE)                                      // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {                                                                    
        if (cnt == UART1_RX_BUF_SIZE)                                    // ��ǰ֡�ѽ��յ��ֽ����������������Ĵ�С; Ϊ�������������������յ�������ֱ������;
        {
            printf("���棺UART1��֡���������ѳ������ջ����С��\r�޸��ļ�bsp_UART.h��UART1_RX_BUF_SIZEֵ,�ɽ�������⣡\r");
            USART1->DR;                                                  // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;
        }
        rxTemp[cnt++] = USART1->DR ;                                     // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ��
        return;
    }

    // �����жϣ������ж�һ֡���ݽ������������ݵ��ⲿ����
    if (USART1->SR & USART_SR_IDLE)                                      // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART1.puRxData, rxTemp, UART1_RX_BUF_SIZE);              // �ѱ�֡���յ������ݣ����뵽�ṹ��������ԱxUARTx.puRxData��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ������ʱβ����0���ַ���������
        xUART1.usRxNum  = cnt;                                           // �ѽ��յ����ֽ��������뵽�ṹ�����xUARTx.usRxNum�У�
        cnt = 0;                                                         // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(rxTemp, 0, UART1_RX_BUF_SIZE);                            // �������ݻ������飬����; ׼����һ�εĽ���
        USART1 ->SR;
        USART1 ->DR;                                                     // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!
        return;
    }

    return;
}

/******************************************************************************
 * ��  ���� UART1_SendData
 * ��  �ܣ� UARTͨ���жϷ�������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע��h�ļ���������ķ���������С��ע������ѹ�뻺�������ٶ��봮�ڷ����ٶȵĳ�ͻ
 * ��  ���� uint8_t*  puData   �跢�����ݵĵ�ַ
 *          uint16_t  usNum    ���͵��ֽ��� ������������h�ļ������õķ��ͻ�������С�궨��
 * ����ֵ�� ��
 ******************************************************************************/
void UART1_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // �����ݷ��뻷�λ�����
    {
        xUART1.puTxFiFoData[xUART1.usTxFiFoData++] = puData[i];  // ���ֽڷŵ�����������λ�ã�Ȼ��ָ�����
        if (xUART1.usTxFiFoData == UART1_TX_BUF_SIZE)            // ���ָ��λ�õ��ﻺ���������ֵ�����0
            xUART1.usTxFiFoData = 0;
    }                                                            // Ϊ�˷����Ķ���⣬����û�аѴ˲��ַ�װ�ɶ��к������������з�װ

    if ((USART1->CR1 & USART_CR1_TXEIE) == 0)                    // ���USART�Ĵ����ķ��ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        USART1->CR1 |= USART_CR1_TXEIE;                          // ��TXEIE�ж�
}

/******************************************************************************
 * ��  ���� UART1_SendString
 * ��  �ܣ� �����ַ���
 *          �÷���ο�printf����ʾ���е�չʾ
 *          ע�⣬������ֽ���Ϊ512-1���ַ������ں������޸�����
 * ��  ���� const char *pcString, ...   (��ͬprintf���÷�)
 * ����ֵ�� ��
 ******************************************************************************/
void UART1_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // ����һ������, ���������������0
    va_list ap;                                             // �½�һ���ɱ�����б�
    va_start(ap, pcString);                                 // �б�ָ���һ���ɱ����
    vsnprintf(mBuffer, 512, pcString, ap);                  // �����в���������ʽ�����������; ����2�������Ʒ��͵�����ֽ���������ﵽ���ޣ���ֻ��������ֵ-1; ���1�ֽ��Զ���'\0'
    va_end(ap);                                             // ��տɱ�����б�
    UART1_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // ���ֽڴ�Ż��λ��壬�Ŷ�׼������
}

/******************************************************************************
 * ��    ���� UART1_SendAT
 * ��    �ܣ� ����AT����, ���ȴ�������Ϣ
 * ��    ���� char     *pcString      ATָ���ַ���
 *            char     *pcAckString   �ڴ���ָ�����Ϣ�ַ���
 *            uint16_t  usTimeOut     ���������ȴ���ʱ�䣬����
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t UART1_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART1_ClearRx();                                              // ��0
    UART1_SendString(pcAT);                                       // ����ATָ���ַ���
    while (usTimeOutMs--)                                         // �ж��Ƿ���ʱ(����ֻ���򵥵�ѭ���жϴ�������
    {
        if (UART1_GetRxNum())                                     // �ж��Ƿ���յ�����
        {
            UART1_ClearRx();                                      // ��0�����ֽ���; ע�⣺���յ����������� ����û�б���0��
            if (strstr((char *)UART1_GetRxData(), pcAckString))   // �жϷ����������Ƿ����ڴ����ַ�
                return 1;                                         // ���أ�0-��ʱû�з��ء�1-���������ڴ�ֵ
        }
        delay_ms(1);                                              // ��ʱ; ���ڳ�ʱ�˳�������������
    }
    return 0;                                                     // ���أ�0-��ʱ�������쳣��1-���������ڴ�ֵ
}


/******************************************************************************
 * ��  ���� UART1_SendStringForDMA
 * ��  �ܣ� UARTͨ��DMA�������ݣ�ʡ��ռ���жϵ�ʱ��
 *         ���ʺϳ������ַ������ֽ����ǳ��࣬
 *         ���� �� �ϡ�1:ֻ�ʺϷ����ַ��������ʺϷ��Ϳ��ܺ�0����ֵ������; 2-ʱ����Ҫ�㹻
 * ��  ���� char strintTemp  Ҫ���͵��ַ����׵�ַ
 * ����ֵ�� ��
 * ��  ע:  ������Ϊ���������������û��ο���Ϊ�˷�����ֲ�����ļ����ⲻ��ʹ�ñ�������
 ******************************************************************************/
#if 0
void UART1_SendStringForDMA(char *stringTemp)
{
    static uint8_t Flag_DmaTxInit = 0;                // ���ڱ���Ƿ�������DMA����
    uint32_t   num = 0;                               // ���͵�������ע�ⷢ�͵ĵ�λ���Ǳ���8λ��
    char *t = stringTemp ;                            // ������ϼ��㷢�͵�����

    while (*t++ != 0)  num++;                         // ����Ҫ���͵���Ŀ���ⲽ�ȽϺ�ʱ�����Է���ÿ��6���ֽڣ�����1us����λ��8λ

    while (DMA1_Channel4->CNDTR > 0);                 // ��Ҫ�����DMA���ڽ����ϴη��ͣ��͵ȴ�; �ý�����ж����־��F4������ô�鷳���������EN�Զ�����
    if (Flag_DmaTxInit == 0)                          // �Ƿ��ѽ��й�USAART_TX��DMA��������
    {
        Flag_DmaTxInit  = 1;                          // ���ñ�ǣ��´ε��ñ������Ͳ��ٽ���������
        USART1 ->CR3   |= 1 << 7;                     // ʹ��DMA����
        RCC->AHBENR    |= 1 << 0;                     // ����DMA1ʱ��  [0]DMA1   [1]DMA2

        DMA1_Channel4->CCR   = 0;                     // ʧ�ܣ� ��0�����Ĵ���, DMA����ʧ�ܲ�������
        DMA1_Channel4->CNDTR = num;                   // ����������
        DMA1_Channel4->CMAR  = (uint32_t)stringTemp;  // �洢����ַ
        DMA1_Channel4->CPAR  = (uint32_t)&USART1->DR; // �����ַ

        DMA1_Channel4->CCR |= 1 << 4;                 // ���ݴ��䷽��   0:�������   1:�Ӵ洢����
        DMA1_Channel4->CCR |= 0 << 5;                 // ѭ��ģʽ       0:��ѭ��     1��ѭ��
        DMA1_Channel4->CCR |= 0 << 6;                 // �����ַ������ģʽ
        DMA1_Channel4->CCR |= 1 << 7;                 // �洢������ģʽ
        DMA1_Channel4->CCR |= 0 << 8;                 // �������ݿ��Ϊ8λ
        DMA1_Channel4->CCR |= 0 << 10;                // �洢�����ݿ��8λ
        DMA1_Channel4->CCR |= 0 << 12;                // �е����ȼ�
        DMA1_Channel4->CCR |= 0 << 14;                // �Ǵ洢�����洢��ģʽ
    }
    DMA1_Channel4->CCR  &= ~((uint32_t)(1 << 0));     // ʧ�ܣ�DMA����ʧ�ܲ�������
    DMA1_Channel4->CNDTR = num;                       // ����������
    DMA1_Channel4->CMAR  = (uint32_t)stringTemp;      // �洢����ַ
    DMA1_Channel4->CCR  |= 1 << 0;                    // ����DMA����
}
#endif

/******************************************************************************
 * ��  ���� UART1_GetRxNum
 * ��  �ܣ� ��ȡ����һ֡���ݵ��ֽ���
 * ��  ���� ��
 * ����ֵ�� 0=û�н��յ����ݣ���0=��һ֡���ݵ��ֽ���
 ******************************************************************************/
uint16_t UART1_GetRxNum(void)
{
    return xUART1.usRxNum ;
}

/******************************************************************************
 * ��  ���� UART1_GetRxData
 * ��  �ܣ� ��ȡ����һ֡���� (���ݵĵ�ַ��
 * ��  ���� ��
 * ����ֵ�� �����ַ(uint8_t*)
 ******************************************************************************/
uint8_t *UART1_GetRxData(void)
{
    return xUART1.puRxData ;
}

/******************************************************************************
 * ��  ���� UART1_ClearRx
 * ��  �ܣ� �������һ֡���ݵĻ���
 *          ��Ҫ����0�ֽ�������Ϊ���������жϽ��յı�׼
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
void UART1_ClearRx(void)
{
    xUART1.usRxNum = 0 ;
}
#endif  // endif UART1_EN





//////////////////////////////////////////////////////////////   UART-2   ///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART2_EN

static xUSATR_TypeDef  xUART2 = { 0 };                      // ���� UART2 ���շ��ṹ��
static uint8_t uaUART2RxData[UART2_RX_BUF_SIZE];            // ���� UART2 �Ľ��ջ���
static uint8_t uaUART2TxFiFoData[UART2_TX_BUF_SIZE];        // ���� UART2 �ķ��ͻ���

/******************************************************************************
 * ��  ���� UART2_Init
 * ��  �ܣ� ��ʼ��USART2��ͨ�����š�Э��������ж����ȼ�
 *          ���ţ�TX-PA2��RX-PA3
 *          Э�飺������-None-8-1
 *          ���ͣ������ж�
 *          ���գ�����+�����ж�
 *
 * ��  ���� uint32_t  ulBaudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/
void UART2_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    // ʹ�� ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);           // ʹ�� GPIOA  ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);          // ʹ�� USART2 ʱ��              
    // ���� ���ŵĸ��ù���                                       
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);       // ����PA2���ù��ܣ�USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);       // ����PA3���ù��ܣ�USART2
    // ���� USART                                                
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);          // ʹ������
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);         // ȡ������
    // ���� TX����                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO ��ʼ���ṹ��
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                     // ���ű�ţ�TX_PA2
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // ���ŷ���: ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // ���ģʽ������
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // ������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // ����ٶȣ�50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� RX����                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;                     // ���ű�ţ�RX_PA3
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� UART                                                   
    USART_InitTypeDef USART_InitStructure;                          // ����USART��ʼ���ṹ��  
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // ���ò�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �����ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ����һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(USART2, &USART_InitStructure);                       // ��ʼ��USART
    // ���� �ж�
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                  // ���� �����ж�
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);                  // ���� �����ж�  
    // ���� �ж����ȼ�
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // �ж����ȼ����ýṹ��
    NVIC_InitStructure .NVIC_IRQChannel = USART2_IRQn;              // ָ���ж�ͨ��
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // ������ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // ������Ӧ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);                                 // ��ʼ��NVIC
    // ������ɣ�����USART
    USART_Cmd(USART2, ENABLE);                                      // ʹ��USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL�� ����
    // ʹ�� ʱ��                                                   
    __HAL_RCC_GPIOA_CLK_ENABLE();                                   // ʹ��GPIOA
    __HAL_RCC_USART2_CLK_ENABLE();                                  // ʹ��USART2
    // ���� USART                                                  
    __HAL_RCC_USART2_FORCE_RESET();                                 // ʹ������
    __HAL_RCC_USART2_RELEASE_RESET();                               // ȡ������    
    // ���� GPIO����                                               
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // ������ʼ��Ҫ�õ��Ľṹ��
    GPIO_InitStruct.Pin   = GPIO_PIN_2 | GPIO_PIN_3;                // ���� TX-PA2��RX-PA3
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // ����ģʽ
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // ������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // ��������
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;                    // ���Ÿ��ù���
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                         // ��ʼ�����Ź���ģʽ
    // ���㲨���ʲ���                                              
    float    temp;                                                 
    uint16_t mantissa, fraction;                                   
    SystemCoreClockUpdate();                                        // ����ϵͳ����Ƶ��ȫ��ֵ; ����SystemCoreClock( )���ڱ�׼�⡢HAL��ͨ��
    temp = (float)(SystemCoreClock / 4) / (ulBaudrate * 16);        // �����ʹ�ʽ����; USART2������APB1, ʱ��Ϊϵͳʱ�ӵ�4��Ƶ; ȫ�ֱ���SystemCoreClock���ڱ�׼�⡢HAL��ͨ��;
    mantissa = temp;				                                // ��������
    fraction = (temp - mantissa) * 16;                              // С������
    USART2 -> BRR  = mantissa << 4 | fraction;                      // ���ò�����
    // ���� UART
    USART2 -> CR1  =   0;                                           // ��0
    USART2 -> CR1 |=   0x01 << 2;                                   // ����ʹ��[02]: 0=ʧ�ܡ�1=ʹ��
    USART2 -> CR1 |=   0x01 << 3;                                   // ����ʹ��[03]��0=ʧ�ܡ�1=ʹ��
    USART2 -> CR1 |=   0x00 << 9;                                   // ��żУ��[09]��0=żEven��1=��Odd;  ע�⣺ʹ����żУ�飬��������Ҫ��1
    USART2 -> CR1 |=   0x00 << 10;                                  // У��λ  [10]��0=���á�1=ʹ��;     ע�⣬ʹ����żУ�飬��λҪ��1 (������Ч�����ݴ���)
    USART2 -> CR1 |=   0x00 << 12;                                  // ����λ  [12]��0=8λ�� 1=9λ;      ע�⣺ʹ����żУ�飬��λҪ��1 (�������ݻᷢ������)
    USART2 -> CR2  =   0;                                           // ������0
    USART2 -> CR2 &= ~(0x03 << 12);                                 // ֹͣλ[13:12]��00b=1��ֹͣλ��01b=0.5��ֹͣλ��10b=2��ֹͣλ��11b=1.5��ֹͣλ
    USART2 -> CR3  =   0;                                           // ������0
    USART2 -> CR3 &= ~(0x01 << 6);                                  // DMA����[6]: 0=��ֹ��1=ʹ��
    USART2 -> CR3 &= ~(0x01 << 7);                                  // DMA����[7]: 0=��ֹ��1=ʹ��
    // ���� �ж�                                                   
    USART2 -> CR1 &= ~(0x01 << 7);                                  // �رշ����ж�
    USART2 -> CR1 |=   0x01 << 5;                                   // ʹ�ܽ����ж�: ���ջ������ǿ�
    USART2 -> CR1 |=   0x01 << 4;                                   // ʹ�ܿ����жϣ�����1�ֽ�ʱ��û�յ�������
    USART2 -> SR   = ~(0x00F0);                                     // �����ж�
    // ���� �ж����ȼ�                                             
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);                        // ����ָ���жϵ���Ӧ���ȼ�;  �������ж������š���ռ���������ȼ�
    HAL_NVIC_EnableIRQ(USART2_IRQn);                                // ʹ�ܡ�����ָ�����ж�
    // ������ɣ�����USART                                         
    USART2 -> CR1 |=   0x01 << 13;                                  // ʹ��UART��ʼ����
#endif                                                             
    // ����������                                                  
    xUART2.puRxData = uaUART2RxData;                                // ��ȡ���ջ������ĵ�ַ
    xUART2.puTxFiFoData = uaUART2TxFiFoData;                        // ��ȡ���ͻ������ĵ�ַ
    // �����ʾ
    printf("UART2 ��ʼ������         %d-None-8-1; ����ɳ�ʼ�����á��շ�����\r", ulBaudrate);
}

/******************************************************************************
 * ��  ���� USART2_IRQHandler
 * ��  �ܣ� USART2�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 * ��  ע�� ���������������ж��¼�ʱ����Ӳ�����á�
 *          ���ʹ�ñ��ļ����룬�ڹ����ļ��������ط���Ҫע��ͬ�������������ͻ��
 ******************************************************************************/
void USART2_IRQHandler(void)
{
    static uint16_t cnt = 0;                                             // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  rxTemp[UART2_RX_BUF_SIZE];                           // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽�ⲿ���棺xUARTx.puRxData[ ]
                                                                         
    // �����жϣ����ڰѻ��λ�������ݣ����ֽڷ���                        
    if ((USART2->SR & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE))  // ��鷢�ͼĴ������ж�ʹ�ܣ��ҷ��ͼĴ���Ϊ��; TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                                                                    
        USART2->DR = xUART2.puTxFiFoData[xUART2.usTxFiFoTail++];         // ��FIFO������ȡ��һ�����ݣ�����USART�ķ��ͼĴ���(Ӳ�����Զ�����)��Ȼ��FIFO��βָ�������ָ����һ��Ҫ���͵�����
        if (xUART2.usTxFiFoTail == UART2_TX_BUF_SIZE)                    // ���FIFOβָ���Ƿ񵽴���FIFO���е�ĩβ
            xUART2.usTxFiFoTail = 0;                                     // ��βָ������Ϊ0��ʵ�ֻ��ζ��еĹ���
        if (xUART2.usTxFiFoTail == xUART2.usTxFiFoData)                  // ���FIFOβָ���Ƿ�׷����ͷָ�룬�����������Ƿ��ѷ������
            USART2->CR1 &= ~USART_CR1_TXEIE;                             // �رշ��ͼĴ������жϣ���ֹ�жϷ�����򱻲���Ҫ�ص���
        return;                                                          
    }                                                                    
                                                                         
    // �����жϣ���������ֽڽ��գ���ŵ���ʱ����                        
    if (USART2->SR & USART_SR_RXNE)                                      // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {                                                                    
        if (cnt == UART2_RX_BUF_SIZE)                                    // ��ǰ֡�ѽ��յ��ֽ����������������Ĵ�С; Ϊ�������������������յ�������ֱ������;
        {
            printf("���棺UART2��֡���������ѳ������ջ����С��\r�޸��ļ�bsp_UART.h��UART2_RX_BUF_SIZEֵ,�ɽ�������⣡\r");
            USART2->DR;                                                  // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;
        }
        rxTemp[cnt++] = USART2->DR ;                                     // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ��
        return;
    }

    // �����жϣ������ж�һ֡���ݽ������������ݵ��ⲿ����
    if (USART2->SR & USART_SR_IDLE)                                      // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART2.puRxData, rxTemp, UART2_RX_BUF_SIZE);              // �ѱ�֡���յ������ݣ����뵽�ṹ��������ԱxUARTx.puRxData��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ������ʱβ����0���ַ���������
        xUART2.usRxNum  = cnt;                                           // �ѽ��յ����ֽ��������뵽�ṹ�����xUARTx.usRxNum�У�
        cnt = 0;                                                         // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(rxTemp, 0, UART2_RX_BUF_SIZE);                            // �������ݻ������飬����; ׼����һ�εĽ���
        USART2 ->SR;
        USART2 ->DR;                                                     // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!
        return;
    }

    return;
}

/******************************************************************************
 * ��  ���� UART2_SendData
 * ��  �ܣ� UARTͨ���жϷ�������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע��h�ļ���������ķ���������С��ע������ѹ�뻺�������ٶ��봮�ڷ����ٶȵĳ�ͻ
 * ��  ���� uint8_t* puData     �跢�����ݵĵ�ַ
 *          uint8_t  usNum      ���͵��ֽ��� ������������h�ļ������õķ��ͻ�������С�궨��
 * ����ֵ�� ��
 ******************************************************************************/
void UART2_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // �����ݷ��뻷�λ�����
    {
        xUART2.puTxFiFoData[xUART2.usTxFiFoData++] = puData[i];  // ���ֽڷŵ�����������λ�ã�Ȼ��ָ�����
        if (xUART2.usTxFiFoData == UART2_TX_BUF_SIZE)            // ���ָ��λ�õ��ﻺ���������ֵ�����0
            xUART2.usTxFiFoData = 0;
    }

    if ((USART2->CR1 & USART_CR1_TXEIE) == 0)                    // ���USART�Ĵ����ķ��ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        USART2->CR1 |= USART_CR1_TXEIE;                          // ��TXEIE�ж�
}

/******************************************************************************
 * ��  ���� UART2_SendString
 * ��  �ܣ� �����ַ���
 *          �÷���ο�printf����ʾ���е�չʾ
 *          ע�⣬������ֽ���Ϊ512-1���ַ������ں������޸�����
 * ��  ���� const char *pcString, ...   (��ͬprintf���÷�)
 * ����ֵ�� ��
 ******************************************************************************/
void UART2_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // ����һ������, ���������������0
    va_list ap;                                             // �½�һ���ɱ�����б�
    va_start(ap, pcString);                                 // �б�ָ���һ���ɱ����
    vsnprintf(mBuffer, 512, pcString, ap);                  // �����в���������ʽ�����������; ����2�������Ʒ��͵�����ֽ���������ﵽ���ޣ���ֻ��������ֵ-1; ���1�ֽ��Զ���'\0'
    va_end(ap);                                             // ��տɱ�����б�
    UART2_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // ���ֽڴ�Ż��λ��壬�Ŷ�׼������
}

/******************************************************************************
 * ��    ���� UART2_SendAT
 * ��    �ܣ� ����AT����, ���ȴ�������Ϣ
 * ��    ���� char     *pcString      ATָ���ַ���
 *            char     *pcAckString   �ڴ���ָ�����Ϣ�ַ���
 *            uint16_t  usTimeOut     ���������ȴ���ʱ�䣬����
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t UART2_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART2_ClearRx();                                              // ��0
    UART2_SendString(pcAT);                                       // ����ATָ���ַ���

    while (usTimeOutMs--)                                         // �ж��Ƿ���ʱ(����ֻ���򵥵�ѭ���жϴ�������
    {
        if (UART2_GetRxNum())                                     // �ж��Ƿ���յ�����
        {
            UART2_ClearRx();                                      // ��0�����ֽ���; ע�⣺���յ����������� ����û�б���0��
            if (strstr((char *)UART2_GetRxData(), pcAckString))   // �жϷ����������Ƿ����ڴ����ַ�
                return 1;                                         // ���أ�0-��ʱû�з��ء�1-���������ڴ�ֵ
        }
        delay_ms(1);                                              // ��ʱ; ���ڳ�ʱ�˳�������������
    }
    return 0;                                                     // ���أ�0-��ʱ�������쳣��1-���������ڴ�ֵ
}

/******************************************************************************
 * ��  ���� UART2_GetRxNum
 * ��  �ܣ� ��ȡ����һ֡���ݵ��ֽ���
 * ��  ���� ��
 * ����ֵ�� 0=û�н��յ����ݣ���0=��һ֡���ݵ��ֽ���
 ******************************************************************************/
uint16_t UART2_GetRxNum(void)
{
    return xUART2.usRxNum ;
}

/******************************************************************************
 * ��  ���� UART2_GetRxData
 * ��  �ܣ� ��ȡ����һ֡���� (���ݵĵ�ַ��
 * ��  ���� ��
 * ����ֵ�� ���ݵĵ�ַ(uint8_t*)
 ******************************************************************************/
uint8_t *UART2_GetRxData(void)
{
    return xUART2.puRxData ;
}

/******************************************************************************
 * ��  ���� UART2_ClearRx
 * ��  �ܣ� �������һ֡���ݵĻ���
 *          ��Ҫ����0�ֽ�������Ϊ���������жϽ��յı�׼
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
void UART2_ClearRx(void)
{
    xUART2.usRxNum = 0 ;
}
#endif  // endif UART2_EN





//////////////////////////////////////////////////////////////   USART-3   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART3_EN

static xUSATR_TypeDef  xUART3 = { 0 };                      // ���� UART3 ���շ��ṹ��
static uint8_t uaUart3RxData[UART3_RX_BUF_SIZE];            // ���� UART3 �Ľ��ջ���
static uint8_t uaUart3TxFiFoData[UART3_TX_BUF_SIZE];        // ���� UART3 �ķ��ͻ���

/******************************************************************************
 * ��  ���� UART3_Init
 * ��  �ܣ� ��ʼ��USART3��ͨ�����š�Э��������ж����ȼ�
 *          ���ţ�TX-PB10��RX-PB11
 *          Э�飺������-None-8-1
 *          ���ͣ������ж�
 *          ���գ�����+�����ж�
 *
 * ��  ���� uint32_t ulBaudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/
void UART3_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    // ʹ�� ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);           // ʹ�� GPIOB  ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);          // ʹ�� USART3 ʱ��              
    // ���� ���ŵĸ��ù���                                       
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);      // ����PB10���ù��ܣ�USART3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);      // ����PB11���ù��ܣ�USART3
    // ���� USART                                                
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);          // ʹ������
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);         // ȡ������
    // ���� TX����                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO ��ʼ���ṹ��
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                    // ���ű��
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // ���ŷ���: ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // ���ģʽ������
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // ������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // ����ٶȣ�50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� RX����                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;                    // ���ű��
    GPIO_Init(GPIOB, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� UART                                                   
    USART_InitTypeDef USART_InitStructure;                          // ����USART��ʼ���ṹ��  
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // ���ò�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �����ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ����һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(USART3, &USART_InitStructure);                       // ��ʼ��USART
    // ���� �ж�
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                  // ���� �����ж�
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);                  // ���� �����ж�  
    // ���� �ж����ȼ�
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // �ж����ȼ����ýṹ��
    NVIC_InitStructure .NVIC_IRQChannel = USART3_IRQn;              // ָ���ж�ͨ��
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // ������ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // ������Ӧ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);                                 // ��ʼ��NVIC
    // ������ɣ�����USART
    USART_Cmd(USART3, ENABLE);                                      // ʹ��USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL�� ����
    // ʹ�� ʱ��                                                    
    __HAL_RCC_GPIOB_CLK_ENABLE();                                   // ʹ��GPIOB
    __HAL_RCC_USART3_CLK_ENABLE();                                  // ʹ��USART3
    // ���� USART                                                   
    __HAL_RCC_USART3_FORCE_RESET();                                 // ʹ������
    __HAL_RCC_USART3_RELEASE_RESET();                               // ȡ������
    // ���� GPIO����                                                
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // ������ʼ��Ҫ�õ��Ľṹ��
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;              // ���� TX-PB10��RX-PB11
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // ����ģʽ
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // ������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // ��������
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;                    // ���Ÿ��ù���
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                         // ��ʼ�����Ź���ģʽ
    // ���㲨���ʲ���                                               
    float    temp;                                                  
    uint16_t mantissa, fraction;                                    
    SystemCoreClockUpdate();                                        // ����ϵͳ����Ƶ��ȫ��ֵ; ����SystemCoreClock( )���ڱ�׼�⡢HAL��ͨ��
    temp = (float)(SystemCoreClock / 4) / (ulBaudrate * 16);        // �����ʹ�ʽ����; USART3������APB1, ʱ��Ϊϵͳʱ�ӵ�4��Ƶ; ȫ�ֱ���SystemCoreClock���ڱ�׼�⡢HAL��ͨ��;
    mantissa = temp;				                                // ��������
    fraction = (temp - mantissa) * 16;                              // С������
    USART3 -> BRR  = mantissa << 4 | fraction;                      // ���ò�����
    // ���� UART
    USART3 -> CR1  =   0;                                           // ��0
    USART3 -> CR1 |=   0x01 << 2;                                   // ����ʹ��[02]: 0=ʧ�ܡ�1=ʹ��
    USART3 -> CR1 |=   0x01 << 3;                                   // ����ʹ��[03]��0=ʧ�ܡ�1=ʹ��
    USART3 -> CR1 |=   0x00 << 9;                                   // ��żУ��[09]��0=żEven��1=��Odd;  ע�⣺ʹ����żУ�飬��������Ҫ��1
    USART3 -> CR1 |=   0x00 << 10;                                  // У��λ  [10]��0=���á�1=ʹ��;     ע�⣬ʹ����żУ�飬��λҪ��1 (������Ч�����ݴ���)
    USART3 -> CR1 |=   0x00 << 12;                                  // ����λ  [12]��0=8λ�� 1=9λ;      ע�⣺ʹ����żУ�飬��λҪ��1 (�������ݻᷢ������)
    USART3 -> CR2  =   0;                                           // ������0
    USART3 -> CR2 &= ~(0x03 << 12);                                 // ֹͣλ[13:12]��00b=1��ֹͣλ��01b=0.5��ֹͣλ��10b=2��ֹͣλ��11b=1.5��ֹͣλ
    USART3 -> CR3  =   0;                                           // ������0
    USART3 -> CR3 &= ~(0x01 << 6);                                  // DMA����[6]: 0=��ֹ��1=ʹ��
    USART3 -> CR3 &= ~(0x01 << 7);                                  // DMA����[7]: 0=��ֹ��1=ʹ��
    // �����ж�                                                     
    USART3 -> CR1 &= ~(0x01 << 7);                                  // �رշ����ж�
    USART3 -> CR1 |=   0x01 << 5;                                   // ʹ�ܽ����ж�: ���ջ������ǿ�
    USART3 -> CR1 |=   0x01 << 4;                                   // ʹ�ܿ����жϣ�����1�ֽ�ʱ��û�յ�������
    USART3 -> SR   = ~(0x00F0);                                     // ����һ���жϱ�־
    // ���� �ж����ȼ�                                              
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);                        // ����ָ���жϵ���Ӧ���ȼ�;  �������ж������š���ռ���������ȼ�
    HAL_NVIC_EnableIRQ(USART3_IRQn);                                // ʹ�ܡ�����ָ�����ж�
    // ������ɣ�����USART                                          
    USART3 -> CR1 |=   0x01 << 13;                                  // ʹ��UART��ʼ����
#endif                                                              
    // ����������                                                   
    xUART3.puRxData = uaUart3RxData;                                // ��ȡ���ջ������ĵ�ַ
    xUART3.puTxFiFoData = uaUart3TxFiFoData;                        // ��ȡ���ͻ������ĵ�ַ
    // �����ʾ
    printf("UART3 ��ʼ������         %d-None-8-1; ����ɳ�ʼ�����á��շ�����\r", ulBaudrate);
}

/******************************************************************************
 * ��  ���� USART3_IRQHandler
 * ��  �ܣ� USART3�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 * ��  ע�� ���������������ж��¼�ʱ����Ӳ�����á�
 *          ���ʹ�ñ��ļ����룬�ڹ����ļ��������ط���Ҫע��ͬ�������������ͻ��
 ******************************************************************************/
void USART3_IRQHandler(void)
{
    static uint16_t cnt = 0;                                             // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  rxTemp[UART3_RX_BUF_SIZE];                           // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽�ⲿ���棺xUARTx.puRxData[ ]
                                                                         
    // �����жϣ����ڰѻ��λ�������ݣ����ֽڷ���                        
    if ((USART3->SR & USART_SR_TXE) && (USART3->CR1 & USART_CR1_TXEIE))  // ��鷢�ͼĴ������ж�ʹ�ܣ��ҷ��ͼĴ���Ϊ��; TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                                                                    
        USART3->DR = xUART3.puTxFiFoData[xUART3.usTxFiFoTail++];         // ��FIFO������ȡ��һ�����ݣ�����USART�ķ��ͼĴ���(Ӳ�����Զ�����)��Ȼ��FIFO��βָ�������ָ����һ��Ҫ���͵�����
        if (xUART3.usTxFiFoTail == UART3_TX_BUF_SIZE)                    // ���FIFOβָ���Ƿ񵽴���FIFO���е�ĩβ
            xUART3.usTxFiFoTail = 0;                                     // ��βָ������Ϊ0��ʵ�ֻ��ζ��еĹ���
        if (xUART3.usTxFiFoTail == xUART3.usTxFiFoData)                  // ���FIFOβָ���Ƿ�׷����ͷָ�룬�����������Ƿ��ѷ������
            USART3->CR1 &= ~USART_CR1_TXEIE;                             // �رշ��ͼĴ������жϣ���ֹ�жϷ�����򱻲���Ҫ�ص���
        return;                                                          
    }                                                                    
                                                                         
    // �����жϣ���������ֽڽ��գ���ŵ���ʱ����                        
    if (USART3->SR & USART_SR_RXNE)                                      // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {                                                                    
        if (cnt == UART3_RX_BUF_SIZE)                                    // ��ǰ֡�ѽ��յ��ֽ����������������Ĵ�С; Ϊ�������������������յ�������ֱ������;
        {
            printf("���棺UART3��֡���������ѳ������ջ����С��\r�޸��ļ�bsp_UART.h��UART3_RX_BUF_SIZEֵ,�ɽ�������⣡\r");
            USART3->DR;                                                  // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;
        }
        rxTemp[cnt++] = USART3->DR ;                                     // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ
        return;
    }

    // �����жϣ������ж�һ֡���ݽ������������ݵ��ⲿ����
    if (USART3->SR & USART_SR_IDLE)                                      // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART3.puRxData, rxTemp, UART3_RX_BUF_SIZE);              // �ѱ�֡���յ������ݣ����뵽�ṹ��������ԱxUARTx.puRxData��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ������ʱβ����0���ַ���������
        xUART3.usRxNum  = cnt;                                           // �ѽ��յ����ֽ��������뵽�ṹ�����xUARTx.usRxNum�У�
        cnt = 0;                                                         // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(rxTemp, 0, UART3_RX_BUF_SIZE);                            // �������ݻ������飬����; ׼����һ�εĽ���
        USART3 ->SR;
        USART3 ->DR;                                                     // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!
        return;
    }

    return;
}

/******************************************************************************
 * ��  ���� UART3_SendData
 * ��  �ܣ� UARTͨ���жϷ�������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע��h�ļ���������ķ���������С��ע������ѹ�뻺�������ٶ��봮�ڷ����ٶȵĳ�ͻ
 * ��  ���� uint8_t* puData   �跢�����ݵĵ�ַ
 *          uint8_t  usNum      ���͵��ֽ��� ������������h�ļ������õķ��ͻ�������С�궨��
 * ����ֵ�� ��
 ******************************************************************************/
void UART3_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                           // �����ݷ��뻷�λ�����
    {
        xUART3.puTxFiFoData[xUART3.usTxFiFoData++] = puData[i];    // ���ֽڷŵ�����������λ�ã�Ȼ��ָ�����
        if (xUART3.usTxFiFoData == UART3_TX_BUF_SIZE)              // ���ָ��λ�õ��ﻺ���������ֵ�����0
            xUART3.usTxFiFoData = 0;
    }

    if ((USART3->CR1 & USART_CR1_TXEIE) == 0)                      // ���USART�Ĵ����ķ��ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        USART3->CR1 |= USART_CR1_TXEIE;                            // ��TXEIE�ж�
}

/******************************************************************************
 * ��  ���� UART3_SendString
 * ��  �ܣ� �����ַ���
 *          �÷���ο�printf����ʾ���е�չʾ
 *          ע�⣬������ֽ���Ϊ512-1���ַ������ں������޸�����
 * ��  ���� const char *pcString, ...   (��ͬprintf���÷�)
 * ����ֵ�� ��
 ******************************************************************************/
void UART3_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                // ����һ������, ���������������0
    va_list ap;                                              // �½�һ���ɱ�����б�
    va_start(ap, pcString);                                  // �б�ָ���һ���ɱ����
    vsnprintf(mBuffer, 512, pcString, ap);                   // �����в���������ʽ�����������; ����2�������Ʒ��͵�����ֽ���������ﵽ���ޣ���ֻ��������ֵ-1; ���1�ֽ��Զ���'\0'
    va_end(ap);                                              // ��տɱ�����б�
    UART3_SendData((uint8_t *)mBuffer, strlen(mBuffer));     // ���ֽڴ�Ż��λ��壬�Ŷ�׼������
}

/******************************************************************************
 * ��    ���� UART3_SendAT
 * ��    �ܣ� ����AT����, ���ȴ�������Ϣ
 * ��    ���� char     *pcString      ATָ���ַ���
 *            char     *pcAckString   �ڴ���ָ�����Ϣ�ַ���
 *            uint16_t  usTimeOut     ���������ȴ���ʱ�䣬����
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t UART3_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART3_ClearRx();                                              // ��0
    UART3_SendString(pcAT);                                       // ����ATָ���ַ���

    while (usTimeOutMs--)                                         // �ж��Ƿ���ʱ(����ֻ���򵥵�ѭ���жϴ�������
    {
        if (UART3_GetRxNum())                                     // �ж��Ƿ���յ�����
        {
            UART3_ClearRx();                                      // ��0�����ֽ���; ע�⣺���յ����������� ����û�б���0��
            if (strstr((char *)UART3_GetRxData(), pcAckString))   // �жϷ����������Ƿ����ڴ����ַ�
                return 1;                                         // ���أ�0-��ʱû�з��ء�1-���������ڴ�ֵ
        }
        delay_ms(1);                                              // ��ʱ; ���ڳ�ʱ�˳�������������
    }
    return 0;                                                     // ���أ�0-��ʱ�������쳣��1-���������ڴ�ֵ
}

/******************************************************************************
 * ��  ���� UART3_GetRxNum
 * ��  �ܣ� ��ȡ����һ֡���ݵ��ֽ���
 * ��  ���� ��
 * ����ֵ�� 0=û�н��յ����ݣ���0=��һ֡���ݵ��ֽ���
 ******************************************************************************/
uint16_t UART3_GetRxNum(void)
{
    return xUART3.usRxNum ;
}

/******************************************************************************
 * ��  ���� UART3_GetRxData
 * ��  �ܣ� ��ȡ����һ֡���� (���ݵĵ�ַ��
 * ��  ���� ��
 * ����ֵ�� ���ݵĵ�ַ(uint8_t*)
 ******************************************************************************/
uint8_t *UART3_GetRxData(void)
{
    return xUART3.puRxData ;
}

/******************************************************************************
 * ��  ���� UART3_ClearRx
 * ��  �ܣ� �������һ֡���ݵĻ���
 *          ��Ҫ����0�ֽ�������Ϊ���������жϽ��յı�׼
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
void UART3_ClearRx(void)
{
    xUART3.usRxNum = 0 ;
}
#endif  // endif UART3_EN





//////////////////////////////////////////////////////////////   UART-4   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART4_EN

static xUSATR_TypeDef  xUART4 = { 0 };                      // ���� UART4 ���շ��ṹ��
static uint8_t uaUart4RxData[UART4_RX_BUF_SIZE];            // ���� UART4 �Ľ��ջ���
static uint8_t uaUart4TxFiFoData[UART4_TX_BUF_SIZE];        // ���� UART4 �ķ��ͻ���

/******************************************************************************
 * ��  ���� UART4_Init
 * ��  �ܣ� ��ʼ��UART4��ͨ�����š�Э��������ж����ȼ�
 *          ���ţ�TX-PC10��RX-PC11
 *          Э�飺������-None-8-1
 *          ���ͣ������ж�
 *          ���գ�����+�����ж�
 *
 * ��  ���� uint32_t ulBaudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/
void UART4_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    // ʹ�� ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);           // ʹ�� GPIOC ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);           // ʹ�� UART4 ʱ��              
    // ���� ���Ÿ��ù���                                       
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);       // ����PC10���ù��ܣ�UART4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);       // ����PC11���ù��ܣ�UART4
    // ���� USART                                                
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, ENABLE);           // ʹ������
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, DISABLE);          // ȡ������
    // ���� TX����                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO ��ʼ���ṹ��
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                    // ���ű��
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // ���ŷ���: ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // ���ģʽ������
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // ������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // ����ٶȣ�50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� RX����                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;                    // ���ű��
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� UART                                                   
    USART_InitTypeDef USART_InitStructure;                          // ����USART��ʼ���ṹ��  
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // ���ò�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �����ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ����һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(UART4, &USART_InitStructure);                        // ��ʼ��USART       
    // ���� �ж�
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);                   // ���� �����ж�
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);                   // ���� �����ж�  
    // ���� �ж����ȼ�
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // �ж����ȼ����ýṹ��
    NVIC_InitStructure .NVIC_IRQChannel = UART4_IRQn;               // ָ���ж�ͨ��
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // ������ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // ������Ӧ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);                                 // ��ʼ��NVIC
    // ������ɣ�����USART
    USART_Cmd(UART4, ENABLE);                                       // ʹ��USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL�� ����
    // ʹ�� ʱ��                                                   
    __HAL_RCC_GPIOC_CLK_ENABLE();                                   // ʹ��GPIOC
    __HAL_RCC_UART4_CLK_ENABLE();                                   // ʹ��UART4
    // ���� USART                                                  
    __HAL_RCC_UART4_FORCE_RESET();                                  // ʹ������
    __HAL_RCC_UART4_RELEASE_RESET();                                // ȡ������
    // ���� GPIO����                                               
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // ������ʼ��Ҫ�õ��Ľṹ��
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;              // ���� TX-PC10��RX-PC11
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // ����ģʽ
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // ������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // ��������
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;                     // ���Ÿ��ù���
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                         // ��ʼ�����Ź���ģʽ
    // ���㲨���ʲ���                                              
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // ����ϵͳ����Ƶ��ȫ��ֵ; ����SystemCoreClock( )���ڱ�׼�⡢HAL��ͨ��
    temp = (float)(SystemCoreClock / 4) / (ulBaudrate * 16);        // �����ʹ�ʽ����; UART4������APB1, ʱ��Ϊϵͳʱ�ӵ�4��Ƶ; ȫ�ֱ���SystemCoreClock���ڱ�׼�⡢HAL��ͨ��;
    mantissa = temp;				                                // ��������
    fraction = (temp - mantissa) * 16;                              // С������
    UART4 -> BRR  = mantissa << 4 | fraction;                       // ���ò�����
    // ���� UART                                                   
    UART4 -> CR1  =   0;                                            // ��0
    UART4 -> CR1 |=   0x01 << 2;                                    // ����ʹ��[02]: 0=ʧ�ܡ�1=ʹ��
    UART4 -> CR1 |=   0x01 << 3;                                    // ����ʹ��[03]��0=ʧ�ܡ�1=ʹ��
    UART4 -> CR1 |=   0x00 << 9;                                    // ��żУ��[09]��0=żEven��1=��Odd;  ע�⣺ʹ����żУ�飬��������Ҫ��1
    UART4 -> CR1 |=   0x00 << 10;                                   // У��λ  [10]��0=���á�1=ʹ��;     ע�⣬ʹ����żУ�飬��λҪ��1 (������Ч�����ݴ���)
    UART4 -> CR1 |=   0x00 << 12;                                   // ����λ  [12]��0=8λ�� 1=9λ;      ע�⣺ʹ����żУ�飬��λҪ��1 (�������ݻᷢ������)
    UART4 -> CR2  =   0;                                            // ������0
    UART4 -> CR2 &= ~(0x03 << 12);                                  // ֹͣλ[13:12]��00b=1��ֹͣλ��01b=0.5��ֹͣλ��10b=2��ֹͣλ��11b=1.5��ֹͣλ
    UART4 -> CR3  =   0;                                            // ������0
    UART4 -> CR3 &= ~(0x01 << 6);                                   // DMA����[6]: 0=��ֹ��1=ʹ��
    UART4 -> CR3 &= ~(0x01 << 7);                                   // DMA����[7]: 0=��ֹ��1=ʹ��
    // ���� �ж�                                                   
    UART4 -> CR1 &= ~(0x01 << 7);                                   // �رշ����ж�
    UART4 -> CR1 |=   0x01 << 5;                                    // ʹ�ܽ����ж�: ���ջ������ǿ�
    UART4 -> CR1 |=   0x01 << 4;                                    // ʹ�ܿ����жϣ�����1�ֽ�ʱ��û�յ�������
    UART4 -> SR   = ~(0x00F0);                                      // �����ж�
    // ���� �ж���ѡ��                                             
    HAL_NVIC_SetPriority(UART4_IRQn, 1, 1);                         // ����ָ���жϵ���Ӧ���ȼ�;  �������ж������š���ռ���������ȼ�
    HAL_NVIC_EnableIRQ(UART4_IRQn);                                 // ʹ�ܡ�����ָ�����ж�
    // ������ɣ��򿪴���                                                    
    UART4 -> CR1 |=   0x01 << 13;                                   // ʹ��UART��ʼ����
#endif                                                             
    // ����������                                                  
    xUART4.puRxData = uaUart4RxData;                                // ��ȡ���ջ������ĵ�ַ
    xUART4.puTxFiFoData = uaUart4TxFiFoData;                        // ��ȡ���ͻ������ĵ�ַ
    // �����ʾ
    printf("UART4 ��ʼ������         %d-None-8-1; ����ɳ�ʼ�����á��շ�����\r", ulBaudrate);
}

/******************************************************************************
 * ��  ���� UART4_IRQHandler
 * ��  �ܣ� UART4���жϴ�����
 *          �����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 * ��  ע�� ���������������ж��¼�ʱ����Ӳ�����á�
 *          ���ʹ�ñ��ļ����룬�ڹ����ļ��������ط���Ҫע��ͬ�������������ͻ��
 ******************************************************************************/
void UART4_IRQHandler(void)
{
    static uint16_t cnt = 0;                                           // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  rxTemp[UART4_RX_BUF_SIZE];                         // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽�ⲿ���棺xUARTx.puRxData[ ]
                                                                       
    // �����жϣ����ڰѻ��λ�������ݣ����ֽڷ���                      
    if ((UART4->SR & USART_SR_TXE) && (UART4->CR1 & USART_CR1_TXEIE))  // ��鷢�ͼĴ������ж�ʹ�ܣ��ҷ��ͼĴ���Ϊ��; TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                                                                  
        UART4->DR = xUART4.puTxFiFoData[xUART4.usTxFiFoTail++];        // ��FIFO������ȡ��һ�����ݣ�����USART�ķ��ͼĴ���(Ӳ�����Զ�����)��Ȼ��FIFO��βָ�������ָ����һ��Ҫ���͵�����
        if (xUART4.usTxFiFoTail == UART4_TX_BUF_SIZE)                  // ���FIFOβָ���Ƿ񵽴���FIFO���е�ĩβ
            xUART4.usTxFiFoTail = 0;                                   // ��βָ������Ϊ0��ʵ�ֻ��ζ��еĹ���
        if (xUART4.usTxFiFoTail == xUART4.usTxFiFoData)                // ���FIFOβָ���Ƿ�׷����ͷָ�룬�����������Ƿ��ѷ������
            UART4->CR1 &= ~USART_CR1_TXEIE;                            // �رշ��ͼĴ������жϣ���ֹ�жϷ�����򱻲���Ҫ�ص���
        return;                                                        
    }                                                                  
                                                                       
    // �����жϣ���������ֽڽ��գ���ŵ���ʱ����                      
    if (UART4->SR & USART_SR_RXNE)                                     // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {                                                                  
        if (cnt == UART4_RX_BUF_SIZE)                                  // ��ǰ֡�ѽ��յ��ֽ����������������Ĵ�С; Ϊ�������������������յ�������ֱ������;
        {           
            printf("���棺UART4��֡���������ѳ������ջ����С��\r�޸��ļ�bsp_UART.h��UART4_RX_BUF_SIZEֵ,�ɽ�������⣡\r");
            UART4->DR;                                                 // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;                                                    
        }                                                              
        rxTemp[cnt++] = UART4->DR ;                                    // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ
        return;                                                        
    }                                                                  
                                                                       
    // �����жϣ������ж�һ֡���ݽ������������ݵ��ⲿ����              
    if (UART4->SR & USART_SR_IDLE)                                     // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {                                                                  
        memcpy(xUART4.puRxData, rxTemp, UART4_RX_BUF_SIZE);            // �ѱ�֡���յ������ݣ����뵽�ṹ��������ԱxUARTx.puRxData��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ������ʱβ����0���ַ���������
        xUART4.usRxNum  = cnt;                                         // �ѽ��յ����ֽ��������뵽�ṹ�����xUARTx.usRxNum�У�
        cnt = 0;                                                       // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(rxTemp, 0, UART4_RX_BUF_SIZE);                          // �������ݻ������飬����; ׼����һ�εĽ���
        UART4 ->SR;                                                    
        UART4 ->DR;                                                    // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!
        return;                                                        
    }

    return;
}

/******************************************************************************
 * ��  ���� UART4_SendData
 * ��  �ܣ� UARTͨ���жϷ�������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע��h�ļ���������ķ���������С��ע������ѹ�뻺�������ٶ��봮�ڷ����ٶȵĳ�ͻ
 * ��  ���� uint8_t* puData   �跢�����ݵĵ�ַ
 *          uint8_t  usNum    ���͵��ֽ��� ������������h�ļ������õķ��ͻ�������С�궨��
 * ����ֵ�� ��
 ******************************************************************************/
void UART4_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // �����ݷ��뻷�λ�����
    {
        xUART4.puTxFiFoData[xUART4.usTxFiFoData++] = puData[i];  // ���ֽڷŵ�����������λ�ã�Ȼ��ָ�����
        if (xUART4.usTxFiFoData == UART4_TX_BUF_SIZE)            // ���ָ��λ�õ��ﻺ���������ֵ�����0
            xUART4.usTxFiFoData = 0;
    }

    if ((UART4->CR1 & USART_CR1_TXEIE) == 0)                     // ���USART�Ĵ����ķ��ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        UART4->CR1 |= USART_CR1_TXEIE;                           // ��TXEIE�ж�
}

/******************************************************************************
 * ��  ���� UART4_SendString
 * ��  �ܣ� �����ַ���
 *          �÷���ο�printf����ʾ���е�չʾ
 *          ע�⣬������ֽ���Ϊ512-1���ַ������ں������޸�����
 * ��  ���� const char *pcString, ...   (��ͬprintf���÷�)
 * ����ֵ�� ��
 ******************************************************************************/
void UART4_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                // ����һ������, ���������������0
    va_list ap;                                              // �½�һ���ɱ�����б�
    va_start(ap, pcString);                                  // �б�ָ���һ���ɱ����
    vsnprintf(mBuffer, 512, pcString, ap);                   // �����в���������ʽ�����������; ����2�������Ʒ��͵�����ֽ���������ﵽ���ޣ���ֻ��������ֵ-1; ���1�ֽ��Զ���'\0'
    va_end(ap);                                              // ��տɱ�����б�
    UART4_SendData((uint8_t *)mBuffer, strlen(mBuffer));     // ���ֽڴ�Ż��λ��壬�Ŷ�׼������
}

/******************************************************************************
 * ��    ���� UART4_SendAT
 * ��    �ܣ� ����AT����, ���ȴ�������Ϣ
 * ��    ���� char     *pcString      ATָ���ַ���
 *            char     *pcAckString   �ڴ���ָ�����Ϣ�ַ���
 *            uint16_t  usTimeOut     ���������ȴ���ʱ�䣬����
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t UART4_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART4_ClearRx();                                              // ��0
    UART4_SendString(pcAT);                                       // ����ATָ���ַ���

    while (usTimeOutMs--)                                         // �ж��Ƿ���ʱ(����ֻ���򵥵�ѭ���жϴ�������
    {
        if (UART4_GetRxNum())                                     // �ж��Ƿ���յ�����
        {
            UART4_ClearRx();                                      // ��0�����ֽ���; ע�⣺���յ����������� ����û�б���0��
            if (strstr((char *)UART4_GetRxData(), pcAckString))   // �жϷ����������Ƿ����ڴ����ַ�
                return 1;                                         // ���أ�0-��ʱû�з��ء�1-���������ڴ�ֵ
        }
        delay_ms(1);                                              // ��ʱ; ���ڳ�ʱ�˳�������������
    }
    return 0;                                                     // ���أ�0-��ʱ�������쳣��1-���������ڴ�ֵ
}

/******************************************************************************
 * ��  ���� UART4_GetRxNum
 * ��  �ܣ� ��ȡ����һ֡���ݵ��ֽ���
 * ��  ���� ��
 * ����ֵ�� 0=û�н��յ����ݣ���0=��һ֡���ݵ��ֽ���
 ******************************************************************************/
uint16_t UART4_GetRxNum(void)
{
    return xUART4.usRxNum ;
}

/******************************************************************************
 * ��  ���� UART4_GetRxData
 * ��  �ܣ� ��ȡ����һ֡���� (���ݵĵ�ַ��
 * ��  ���� ��
 * ����ֵ�� ���ݵĵ�ַ(uint8_t*)
 ******************************************************************************/
uint8_t *UART4_GetRxData(void)
{
    return xUART4.puRxData ;
}

/******************************************************************************
 * ��  ���� UART4_ClearRx
 * ��  �ܣ� �������һ֡���ݵĻ���
 *          ��Ҫ����0�ֽ�������Ϊ���������жϽ��յı�׼
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
void UART4_ClearRx(void)
{
    xUART4.usRxNum = 0 ;
}
#endif  // endif UART4_EN




//////////////////////////////////////////////////////////////   UART-5   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART5_EN

static xUSATR_TypeDef  xUART5 = { 0 };                      // ���� UART5 ���շ��ṹ��
static uint8_t uaUart5RxData[UART5_RX_BUF_SIZE];            // ���� UART5 �Ľ��ջ���
static uint8_t uaUart5TxFiFoData[UART5_TX_BUF_SIZE];        // ���� UART5 �ķ��ͻ���

/******************************************************************************
 * ��  ���� UART5_Init
 * ��  �ܣ� ��ʼ��UART5��ͨ�����š�Э��������ж����ȼ�
 *          ���ţ�TX-PC12��RX-PD2
 *          Э�飺������-None-8-1
 *          ���ͣ������ж�
 *          ���գ�����+�����ж�
 *
 * ��  ���� uint32_t ulBaudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/
void UART5_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    // ʹ�� ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);           // ʹ�� GPIOC ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);           // ʹ�� GPIOD ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);           // ʹ�� UART5 ʱ��              
    // ���� ���Ÿ��ù���                                       
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);       // ����PC12���ù��ܣ�UART5
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);        // ����PD2���ù��� ��UART5
    // ���� USART                                                
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);           // ʹ������
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, DISABLE);          // ȡ������
    // ���� TX����                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO ��ʼ���ṹ��
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;                    // ���ű��
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // ���ŷ���: ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // ���ģʽ������
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // ������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // ����ٶȣ�50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� RX����                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                     // ���ű��
    GPIO_Init(GPIOD, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ����UART                                                   
    USART_InitTypeDef USART_InitStructure;                          // ����USART��ʼ���ṹ��  
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // ���ò�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �����ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ����һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(UART5, &USART_InitStructure);                        // ��ʼ��USART
    // ���� �ж�
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);                   // ���� �����ж�
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);                   // ���� �����ж�  
    // �ж����ȼ�����
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // �ж����ȼ����ýṹ��
    NVIC_InitStructure .NVIC_IRQChannel = UART5_IRQn;               // ָ���ж�ͨ��
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // ������ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // ������Ӧ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);                                 // ��ʼ��NVIC
    // ������ɣ�����USART
    USART_Cmd(UART5, ENABLE);                                       // ʹ��USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL�� ����
    // ʹ�� ʱ��                                                    
    __HAL_RCC_GPIOC_CLK_ENABLE();                                   // ʹ��GPIOC 
    __HAL_RCC_GPIOD_CLK_ENABLE();                                   // ʹ��GPIOD 
    __HAL_RCC_UART5_CLK_ENABLE();                                   // ʹ��UART5
    // ���� UART                                                    
    __HAL_RCC_UART5_FORCE_RESET();                                  // ʹ������
    __HAL_RCC_UART5_RELEASE_RESET();                                // ȡ������
    // ���� TX����                                                  
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // ������ʼ��Ҫ�õ��Ľṹ��
    GPIO_InitStruct.Pin   = GPIO_PIN_12 ;                           // ���� TX-PC12
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // ����ģʽ
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                            // ������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // ��������
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;                     // ���Ÿ��ù���
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                         // ��ʼ������
    // ���� RX����                                                  
    GPIO_InitStruct.Pin   = GPIO_PIN_2 ;                            // ���� RX-PD2
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);                         // ��ʼ�����Ź���ģʽ
    // ���㲨���ʲ���                                               
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // ����ϵͳ����Ƶ��ȫ��ֵ; ����SystemCoreClock( )���ڱ�׼�⡢HAL��ͨ��
    temp = (float)(SystemCoreClock / 4) / (ulBaudrate * 16);        // �����ʹ�ʽ����; UART5������APB1, ʱ��Ϊϵͳʱ�ӵ�4��Ƶ; ȫ�ֱ���SystemCoreClock���ڱ�׼�⡢HAL��ͨ��;
    mantissa = temp;				                                // ��������
    fraction = (temp - mantissa) * 16;                              // С������
    UART5 -> BRR  = mantissa << 4 | fraction;                       // ���ò�����
    // ���� UART                                                   
    UART5 -> CR1  =   0;                                            // ��0
    UART5 -> CR1 |=   0x01 << 2;                                    // ����ʹ��[02]: 0=ʧ�ܡ�1=ʹ��
    UART5 -> CR1 |=   0x01 << 3;                                    // ����ʹ��[03]��0=ʧ�ܡ�1=ʹ��
    UART5 -> CR1 |=   0x00 << 9;                                    // ��żУ��[09]��0=żEven��1=��Odd;  ע�⣺ʹ����żУ�飬��������Ҫ��1
    UART5 -> CR1 |=   0x00 << 10;                                   // У��λ  [10]��0=���á�1=ʹ��;     ע�⣬ʹ����żУ�飬��λҪ��1 (������Ч�����ݴ���)
    UART5 -> CR1 |=   0x00 << 12;                                   // ����λ  [12]��0=8λ�� 1=9λ;      ע�⣺ʹ����żУ�飬��λҪ��1 (�������ݻᷢ������)
    UART5 -> CR2  =   0;                                            // ������0
    UART5 -> CR2 &= ~(0x03 << 12);                                  // ֹͣλ[13:12]��00b=1��ֹͣλ��01b=0.5��ֹͣλ��10b=2��ֹͣλ��11b=1.5��ֹͣλ
    UART5 -> CR3  =   0;                                            // ������0
    UART5 -> CR3 &= ~(0x01 << 6);                                   // DMA����[6]: 0=��ֹ��1=ʹ��
    UART5 -> CR3 &= ~(0x01 << 7);                                   // DMA����[7]: 0=��ֹ��1=ʹ��
    // ���� �ж�                                                   
    UART5 -> CR1 &= ~(0x01 << 7);                                   // �رշ����ж�
    UART5 -> CR1 |=   0x01 << 5;                                    // ʹ�ܽ����ж�: ���ջ������ǿ�
    UART5 -> CR1 |=   0x01 << 4;                                    // ʹ�ܿ����жϣ�����1�ֽ�ʱ��û�յ�������
    UART5 -> SR   = ~(0x00F0);                                      // �����ж�
    // ���� �ж����ȼ�                                             
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);                         // ����ָ���жϵ���Ӧ���ȼ�;  �������ж������š���ռ���������ȼ�
    HAL_NVIC_EnableIRQ(UART5_IRQn);                                 // ʹ�ܡ�����ָ�����ж�
    // ������ɣ��򿪴���                                          
    UART5 -> CR1 |=   0x01 << 13;                                   // ʹ��UART��ʼ����    
#endif

    // ����������
    xUART5.puRxData = uaUart5RxData;                                // ��ȡ���ջ������ĵ�ַ
    xUART5.puTxFiFoData = uaUart5TxFiFoData;                        // ��ȡ���ͻ������ĵ�ַ
    // �����ʾ
    printf("UART5 ��ʼ������         %d-None-8-1; ����ɳ�ʼ�����á��շ�����\r", ulBaudrate);
}

/******************************************************************************
 * ��  ���� UART5_IRQHandler
 * ��  �ܣ� UART5�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 * ��  ע�� ���������������ж��¼�ʱ����Ӳ�����á�
 *          ���ʹ�ñ��ļ����룬�ڹ����ļ��������ط���Ҫע��ͬ�������������ͻ��
 ******************************************************************************/
void UART5_IRQHandler(void)
{
    static uint16_t cnt = 0;                                           // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  rxTemp[UART5_RX_BUF_SIZE];                         // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽�ⲿ���棺xUARTx.puRxData[ ]
                                                                       
    // �����жϣ����ڰѻ��λ�������ݣ����ֽڷ���                      
    if ((UART5->SR & USART_SR_TXE) && (UART5->CR1 & USART_CR1_TXEIE))  // ��鷢�ͼĴ������ж�ʹ�ܣ��ҷ��ͼĴ���Ϊ��; TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                                                                  
        UART5->DR = xUART5.puTxFiFoData[xUART5.usTxFiFoTail++];        // ��FIFO������ȡ��һ�����ݣ�����USART�ķ��ͼĴ���(Ӳ�����Զ�����)��Ȼ��FIFO��βָ�������ָ����һ��Ҫ���͵�����
        if (xUART5.usTxFiFoTail == UART5_TX_BUF_SIZE)                  // ���FIFOβָ���Ƿ񵽴���FIFO���е�ĩβ
            xUART5.usTxFiFoTail = 0;                                   // ��βָ������Ϊ0��ʵ�ֻ��ζ��еĹ���
        if (xUART5.usTxFiFoTail == xUART5.usTxFiFoData)                // ���FIFOβָ���Ƿ�׷����ͷָ�룬�����������Ƿ��ѷ������
            UART5->CR1 &= ~USART_CR1_TXEIE;                            // �رշ��ͼĴ������жϣ���ֹ�жϷ�����򱻲���Ҫ�ص���
        return;                                                        
    }                                                                  
                                                                       
    // �����жϣ���������ֽڽ��գ���ŵ���ʱ����                      
    if (UART5->SR & USART_SR_RXNE)                                     // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {                                                                  
        if (cnt == UART5_RX_BUF_SIZE)                                  // ��ǰ֡�ѽ��յ��ֽ����������������Ĵ�С; Ϊ�������������������յ�������ֱ������;
        {
            printf("���棺UART5��֡���������ѳ������ջ����С��\r�޸��ļ�bsp_UART.h��UART5_RX_BUF_SIZEֵ,�ɽ�������⣡\r");
            UART5->DR;                                                 // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;                                                   
        }                                                             
        rxTemp[cnt++] = UART5->DR ;                                    // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ
        return;                                                       
    }                                                                 
                                                                      
    // �����жϣ������ж�һ֡���ݽ������������ݵ��ⲿ����             
    if (UART5->SR & USART_SR_IDLE)                                     // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {                                                                 
        memcpy(xUART5.puRxData, rxTemp, UART5_RX_BUF_SIZE);            // �ѱ�֡���յ������ݣ����뵽�ṹ��������ԱxUARTx.puRxData��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ������ʱβ����0���ַ���������
        xUART5.usRxNum  = cnt;                                         // �ѽ��յ����ֽ��������뵽�ṹ�����xUARTx.usRxNum�У�
        cnt = 0;                                                       // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(rxTemp, 0, UART5_RX_BUF_SIZE);                          // �������ݻ������飬����; ׼����һ�εĽ���
        UART5 -> SR;                                                   
        UART5 -> DR;                                                   // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!
        return;                                                       
    }

    return;
}

/******************************************************************************
 * ��  ���� UART5_SendData
 * ��  �ܣ� UARTͨ���жϷ�������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע��h�ļ���������ķ���������С��ע������ѹ�뻺�������ٶ��봮�ڷ����ٶȵĳ�ͻ
 * ��  ���� uint8_t* pudata     �跢�����ݵĵ�ַ
 *          uint8_t  usNum      ���͵��ֽ��� ������������h�ļ������õķ��ͻ�������С�궨��
 * ����ֵ�� ��
 ******************************************************************************/
void UART5_SendData(uint8_t *pudata, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // �����ݷ��뻷�λ�����
    {
        xUART5.puTxFiFoData[xUART5.usTxFiFoData++] = pudata[i];  // ���ֽڷŵ�����������λ�ã�Ȼ��ָ�����
        if (xUART5.usTxFiFoData == UART5_TX_BUF_SIZE)            // ���ָ��λ�õ��ﻺ���������ֵ�����0
            xUART5.usTxFiFoData = 0;
    }
                                                                
    if ((UART5->CR1 & USART_CR1_TXEIE) == 0)                     // ���USART�Ĵ����ķ��ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        UART5->CR1 |= USART_CR1_TXEIE;                           // ��TXEIE�ж�
}

/******************************************************************************
 * ��  ���� UART5_SendString
 * ��  �ܣ� �����ַ���
 *          �÷���ο�printf����ʾ���е�չʾ
 *          ע�⣬������ֽ���Ϊ512-1���ַ������ں������޸�����
 * ��  ���� const char *pcString, ...   (��ͬprintf���÷�)
 * ����ֵ�� ��
 ******************************************************************************/
void UART5_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // ����һ������, ���������������0
    va_list ap;                                             // �½�һ���ɱ�����б�
    va_start(ap, pcString);                                 // �б�ָ���һ���ɱ����
    vsnprintf(mBuffer, 512, pcString, ap);                  // �����в���������ʽ�����������; ����2�������Ʒ��͵�����ֽ���������ﵽ���ޣ���ֻ��������ֵ-1; ���1�ֽ��Զ���'\0'
    va_end(ap);                                             // ��տɱ�����б�
    UART5_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // ���ֽڴ�Ż��λ��壬�Ŷ�׼������
}

/******************************************************************************
 * ��    ���� UART5_SendAT
 * ��    �ܣ� ����AT����, ���ȴ�������Ϣ
 * ��    ���� char     *pcString      ATָ���ַ���
 *            char     *pcAckString   �ڴ���ָ�����Ϣ�ַ���
 *            uint16_t  usTimeOut     ���������ȴ���ʱ�䣬����
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t UART5_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART5_ClearRx();                                              // ��0
    UART5_SendString(pcAT);                                       // ����ATָ���ַ���

    while (usTimeOutMs--)                                         // �ж��Ƿ���ʱ(����ֻ���򵥵�ѭ���жϴ�������
    {
        if (UART5_GetRxNum())                                     // �ж��Ƿ���յ�����
        {
            UART5_ClearRx();                                      // ��0�����ֽ���; ע�⣺���յ����������� ����û�б���0��
            if (strstr((char *)UART5_GetRxData(), pcAckString))   // �жϷ����������Ƿ����ڴ����ַ�
                return 1;                                         // ���أ�0-��ʱû�з��ء�1-���������ڴ�ֵ
        }
        delay_ms(1);                                              // ��ʱ; ���ڳ�ʱ�˳�������������
    }
    return 0;                                                     // ���أ�0-��ʱ�������쳣��1-���������ڴ�ֵ
}

/******************************************************************************
 * ��  ���� UART5_GetRxNum
 * ��  �ܣ� ��ȡ����һ֡���ݵ��ֽ���
 * ��  ���� ��
 * ����ֵ�� 0=û�н��յ����ݣ���0=��һ֡���ݵ��ֽ���
 ******************************************************************************/
uint16_t UART5_GetRxNum(void)
{
    return xUART5.usRxNum ;
}

/******************************************************************************
 * ��  ���� UART5_GetRxData
 * ��  �ܣ� ��ȡ����һ֡���� (���ݵĵ�ַ��
 * ��  ���� ��
 * ����ֵ�� ���ݵĵ�ַ(uint8_t*)
 ******************************************************************************/
uint8_t *UART5_GetRxData(void)
{
    return xUART5.puRxData ;
}

/******************************************************************************
 * ��  ���� UART5_ClearRx
 * ��  �ܣ� �������һ֡���ݵĻ���
 *          ��Ҫ����0�ֽ�������Ϊ���������жϽ��յı�׼
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
void UART5_ClearRx(void)
{
    xUART5.usRxNum = 0 ;
}
#endif  // endif UART5_EN




//////////////////////////////////////////////////////////////   USART-6   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART6_EN

static xUSATR_TypeDef  xUART6 = { 0 };                      // ���� UART6 ���շ��ṹ��
static uint8_t uaUart6RxData[UART6_RX_BUF_SIZE];            // ���� UART6 �Ľ��ջ���
static uint8_t uaUart6TxFiFoData[UART6_TX_BUF_SIZE];        // ���� UART6 �ķ��ͻ���

/******************************************************************************
 * ��  ���� UART6_Init
 * ��  �ܣ� ��ʼ��USART6��ͨ�����š�Э��������ж����ȼ�
 *          ���ţ�TX-PC6��RX-PC7
 *          Э�飺������-None-8-1
 *          ���ͣ������ж�
 *          ���գ�����+�����ж�
 *
 * ��  ���� uint32_t ulBaudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/
void UART6_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    // ʹ�� ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);           // ʹ�� GPIOC  ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);          // ʹ�� USART6 ʱ��              
    // ���� ���Ÿ��ù���                                       
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);       // ����PC6���ù��ܣ�USART6
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);       // ����PC7���ù��ܣ�USART6
    // ���� USART                                                
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);          // ʹ������
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);         // ȡ������
    // ���� TX����                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO ��ʼ���ṹ��
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;                     // ���ű��
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // ���ŷ���: ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // ���ģʽ������
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // ������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // ����ٶȣ�50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ���� RX����                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;                     // ���ű��
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // ��ʼ�������������������µ�оƬ�Ĵ���
    // ����UART                                                   
    USART_InitTypeDef USART_InitStructure;                          // ����USART��ʼ���ṹ��  
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // ���ò�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �����ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ����һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // �շ�ģʽ
    USART_Init(USART6, &USART_InitStructure);                       // ��ʼ��USART
    // ���� �ж�
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);                  // ���� �����ж�
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);                  // ���� �����ж�  
    // �ж����ȼ�����
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // �ж����ȼ����ýṹ��
    NVIC_InitStructure .NVIC_IRQChannel = USART6_IRQn;              // ָ���ж�ͨ��
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // ������ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // ������Ӧ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);                                 // ��ʼ��NVIC
    // ������ɣ�����USART
    USART_Cmd(USART6, ENABLE);                                      // ʹ��USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL�� ����
    // ʹ�� ʱ��                                                 
    __HAL_RCC_GPIOC_CLK_ENABLE();                                   // ʹ��GPIOC 
    __HAL_RCC_USART6_CLK_ENABLE();                                  // ʹ��UART6
    // ���� UART5                                                
    __HAL_RCC_USART6_FORCE_RESET();                                 // ʹ������
    __HAL_RCC_USART6_RELEASE_RESET();                               // ȡ������
    // ���� GPIO����                                      
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // ������ʼ��Ҫ�õ��Ľṹ��
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;                // ���� TX-PC6��RX-PC7
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // ����ģʽ
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // ������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // ��������
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;                    // ���Ÿ��ù���
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                         // ��ʼ�����Ź���ģʽ
    // ���㲨����
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // ����ϵͳ����Ƶ��ȫ��ֵ; ����SystemCoreClock( )���ڱ�׼�⡢HAL��ͨ��
    temp = (float)(SystemCoreClock / 2) / (ulBaudrate * 16);        // �����ʹ�ʽ����; USART6������APB2, ʱ��Ϊϵͳʱ�ӵ�2��Ƶ; ȫ�ֱ���SystemCoreClock���ڱ�׼�⡢HAL��ͨ��;
    mantissa = temp;				                                // ��������
    fraction = (temp - mantissa) * 16;                              // С������
    USART6 -> BRR  = mantissa << 4 | fraction;                      // ���ò�����
    // ���� UART                                                    
    USART6 -> CR1  =   0;                                           // ��0
    USART6 -> CR1 |=   0x01 << 2;                                   // ����ʹ��[02]: 0=ʧ�ܡ�1=ʹ��
    USART6 -> CR1 |=   0x01 << 3;                                   // ����ʹ��[03]��0=ʧ�ܡ�1=ʹ��
    USART6 -> CR1 |=   0x00 << 9;                                   // ��żУ��[09]��0=żEven��1=��Odd;  ע�⣺ʹ����żУ�飬��������Ҫ��1
    USART6 -> CR1 |=   0x00 << 10;                                  // У��λ  [10]��0=���á�1=ʹ��;     ע�⣬ʹ����żУ�飬��λҪ��1 (������Ч�����ݴ���)
    USART6 -> CR1 |=   0x00 << 12;                                  // ����λ  [12]��0=8λ�� 1=9λ;      ע�⣺ʹ����żУ�飬��λҪ��1 (�������ݻᷢ������)
    USART6 -> CR2  =   0;                                           // ������0
    USART6 -> CR2 &= ~(0x03 << 12);                                 // ֹͣλ[13:12]��00b=1��ֹͣλ��01b=0.5��ֹͣλ��10b=2��ֹͣλ��11b=1.5��ֹͣλ
    USART6 -> CR3  =   0;                                           // ������0
    USART6 -> CR3 &= ~(0x01 << 6);                                  // DMA����[6]: 0=��ֹ��1=ʹ��
    USART6 -> CR3 &= ~(0x01 << 7);                                  // DMA����[7]: 0=��ֹ��1=ʹ��
    // ���� �ж�                                                    
    USART6 -> CR1 &= ~(0x01 << 7);                                  // �رշ����ж�
    USART6 -> CR1 |=   0x01 << 5;                                   // ʹ�ܽ����ж�: ���ջ������ǿ�
    USART6 -> CR1 |=   0x01 << 4;                                   // ʹ�ܿ����жϣ�����1�ֽ�ʱ��û�յ�������
    USART6 -> SR   = ~(0x00F0);                                     // �����ж�
    // ���� �ж����ȼ�                                                
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 1);                        // ����ָ���жϵ���Ӧ���ȼ�;  �������ж������š���ռ���������ȼ�
    HAL_NVIC_EnableIRQ(USART6_IRQn);                                // ʹ�ܡ�����ָ�����ж�
    // ������ɣ��򿪴���                                           
    USART6 -> CR1 |=   0x01 << 13;                                  // ʹ��UART��ʼ����
#endif

    // ����������
    xUART6.puRxData = uaUart6RxData;                                // ��ȡ���ջ������ĵ�ַ
    xUART6.puTxFiFoData = uaUart6TxFiFoData;                        // ��ȡ���ͻ������ĵ�ַ
    // �����ʾ
    printf("UART6 ��ʼ������         %d-None-8-1; ����ɳ�ʼ�����á��շ�����\r", ulBaudrate);
}

/******************************************************************************
 * ��  ���� USART6_IRQHandler
 * ��  �ܣ� USART6�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 * ��  ע�� ���������������ж��¼�ʱ����Ӳ�����á�
 *          ���ʹ�ñ��ļ����룬�ڹ����ļ��������ط���Ҫע��ͬ�������������ͻ��
 ******************************************************************************/
void USART6_IRQHandler(void)
{
    static uint16_t cnt = 0;                                             // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  rxTemp[UART6_RX_BUF_SIZE];                           // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽ȫ�ֱ�����xUARTx.puRxData[xx]�У�

    // �����жϣ����ڰѻ��λ�������ݣ����ֽڷ���
    if ((USART6->SR & USART_SR_TXE) && (USART6->CR1 & USART_CR1_TXEIE))  // ��鷢�ͼĴ������ж�ʹ�ܣ��ҷ��ͼĴ���Ϊ��; TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                                                                    
        USART6->DR = xUART6.puTxFiFoData[xUART6.usTxFiFoTail++];         // ��FIFO������ȡ��һ�����ݣ�����USART�ķ��ͼĴ���(Ӳ�����Զ�����)��Ȼ��FIFO��βָ�������ָ����һ��Ҫ���͵�����
        if (xUART6.usTxFiFoTail == UART6_TX_BUF_SIZE)                    // ���FIFOβָ���Ƿ񵽴���FIFO���е�ĩβ
            xUART6.usTxFiFoTail = 0;                                     // ��βָ������Ϊ0��ʵ�ֻ��ζ��еĹ���
        if (xUART6.usTxFiFoTail == xUART6.usTxFiFoData)                  // ���FIFOβָ���Ƿ�׷����ͷָ�룬�����������Ƿ��ѷ������
            USART6->CR1 &= ~USART_CR1_TXEIE;                             // �رշ��ͼĴ������жϣ���ֹ�жϷ�����򱻲���Ҫ�ص���
        return;                                                          
    }                                                                    
                                                                         
    // �����жϣ���������ֽڽ��գ���ŵ���ʱ����                        
    if (USART6->SR & USART_SR_RXNE)                                      // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {                                                                    
        if (cnt >= UART6_RX_BUF_SIZE)                                    // ��ǰ֡�ѽ��յ��ֽ����������������Ĵ�С; Ϊ�������������������յ�������ֱ������;
        {
            printf("���棺USART6��֡���������ѳ������ջ����С��\r�޸��ļ�bsp_UART.h��UART6_RX_BUF_SIZEֵ,�ɽ�������⣡\r");
            USART6->DR;                                                  // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;                                                      
        }                                                                
        rxTemp[cnt++] = USART6->DR ;                                     // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ
        return;                                                          
    }                                                                    
                                                                         
    // �����жϣ������ж�һ֡���ݽ������������ݵ��ⲿ����                
    if (USART6->SR & USART_SR_IDLE)                                      // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {                                                                    
        memcpy(xUART6.puRxData, rxTemp, UART6_RX_BUF_SIZE);              // �ѱ�֡���յ������ݣ����뵽�ṹ��������ԱxUARTx.puRxData��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ������ʱβ����0���ַ���������
        xUART6.usRxNum  = cnt;                                           // �ѽ��յ����ֽ��������뵽�ṹ�����xUARTx.usRxNum�У�
        cnt = 0;                                                         // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(rxTemp, 0, UART6_RX_BUF_SIZE);                            // �������ݻ������飬����; ׼����һ�εĽ���
        USART6 ->SR;                                                     
        USART6 ->DR;                                                     // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!
        return;
    }

    return;
}


/******************************************************************************
 * ��  ���� UART6_SendData
 * ��  �ܣ� UARTͨ���жϷ�������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע��h�ļ���������ķ���������С��ע������ѹ�뻺�������ٶ��봮�ڷ����ٶȵĳ�ͻ
 * ��  ���� uint8_t  *puData   �跢�����ݵĵ�ַ
 *          uint8_t   usNum    ���͵��ֽ��� ������������h�ļ������õķ��ͻ�������С�궨��
 * ����ֵ�� ��
 ******************************************************************************/
void UART6_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                           // �����ݷ��뻷�λ�����
    {
        xUART6.puTxFiFoData[xUART6.usTxFiFoData++] = puData[i];    // ���ֽڷŵ�����������λ�ã�Ȼ��ָ�����
        if (xUART6.usTxFiFoData == UART6_TX_BUF_SIZE)              // ���ָ��λ�õ��ﻺ���������ֵ�����0
            xUART6.usTxFiFoData = 0;
    }

    if ((USART6->CR1 & USART_CR1_TXEIE) == 0)                      // ���USART�Ĵ����ķ��ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        USART6->CR1 |= USART_CR1_TXEIE;                            // ��TXEIE�ж�
}

/******************************************************************************
 * ��  ���� UART6_SendString
 * ��  �ܣ� �����ַ���
 *          �÷���ο�printf����ʾ���е�չʾ
 *          ע�⣬������ֽ���Ϊ512-1���ַ������ں������޸�����
 * ��  ���� const char *pcString, ...   (��ͬprintf���÷�)
 * ����ֵ�� ��
 ******************************************************************************/
void UART6_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // ����һ������, ���������������0
    va_list ap;                                             // �½�һ���ɱ�����б�
    va_start(ap, pcString);                                 // �б�ָ���һ���ɱ����
    vsnprintf(mBuffer, 512, pcString, ap);                  // �����в���������ʽ�����������; ����2�������Ʒ��͵�����ֽ���������ﵽ���ޣ���ֻ��������ֵ-1; ���1�ֽ��Զ���'\0'
    va_end(ap);                                             // ��տɱ�����б�
    UART6_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // ���ֽڴ�Ż��λ��壬�Ŷ�׼������
}

/******************************************************************************
 * ��    ���� UART6_SendAT
 * ��    �ܣ� ����AT����, ���ȴ�������Ϣ
 * ��    ���� char     *pcString      ATָ���ַ���
 *            char     *pcAckString   �ڴ���ָ�����Ϣ�ַ���
 *            uint16_t  usTimeOut     ���������ȴ���ʱ�䣬����
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t UART6_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART6_ClearRx();                                              // ��0
    UART6_SendString(pcAT);                                       // ����ATָ���ַ���

    while (usTimeOutMs--)                                         // �ж��Ƿ���ʱ(����ֻ���򵥵�ѭ���жϴ�������
    {
        if (UART6_GetRxNum())                                     // �ж��Ƿ���յ�����
        {
            UART6_ClearRx();                                      // ��0�����ֽ���; ע�⣺���յ����������� ����û�б���0��
            if (strstr((char *)UART6_GetRxData(), pcAckString))   // �жϷ����������Ƿ����ڴ����ַ�
                return 1;                                         // ���أ�0-��ʱû�з��ء�1-���������ڴ�ֵ
        }
        delay_ms(1);                                              // ��ʱ; ���ڳ�ʱ�˳�������������
    }
    return 0;                                                     // ���أ�0-��ʱ�������쳣��1-���������ڴ�ֵ
}

/******************************************************************************
 * ��  ���� UART6_GetRxNum
 * ��  �ܣ� ��ȡ����һ֡���ݵ��ֽ���
 * ��  ���� ��
 * ����ֵ�� 0=û�н��յ����ݣ���0=��һ֡���ݵ��ֽ���
 ******************************************************************************/
uint16_t UART6_GetRxNum(void)
{
    return xUART6.usRxNum ;
}

/******************************************************************************
 * ��  ���� UART6_GetRxData
 * ��  �ܣ� ��ȡ����һ֡���� (���ݵĵ�ַ��
 * ��  ���� ��
 * ����ֵ�� ���ݵĵ�ַ(uint8_t*)
 ******************************************************************************/
uint8_t *UART6_GetRxData(void)
{
    return xUART6.puRxData ;
}

/******************************************************************************
 * ��  ���� UART6_ClearRx
 * ��  �ܣ� �������һ֡���ݵĻ���
 *          ��Ҫ����0�ֽ�������Ϊ���������жϽ��յı�׼
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
void UART6_ClearRx(void)
{
    xUART6.usRxNum = 0 ;
}
#endif  // endif UART6_EN






/////////////////////////////////////////////////////////  ModbusCRC16 У��   /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CRC16���ֵ������
// ������ֱ�Ӽ���CRCֵ����Լ�ɼ���һ������ʱ��
static const uint8_t aModbusCRC16[] =      
{
    // ��λ
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    // ��λ
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,  
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};


/**********************************************************************************************************
 * �� ��:  Modbus_AddCRC16
 * �� ��:  ������ĩβ׷�����ֽڵ�ModbusCRC16У��ֵ
 * �� ע�� 1�����������ݴ����ԭ���ݣ�����CRC16ֵ��������ĩβ׷�����ֽڵ�CRC16У��ֵ;
 *         2��ע�⴫��������ַ��Ҫ����Ԥ��2�ֽڵĿռ䣬����׷��CRC16��У��ֵ;
 *         
 * �� ��:  uint8_t*  _pcData���������У��ֵ������
 *         uint16_t  _usLen �������ֽ���; ����Ϊ���ݵ�ԭ�ֽ������ɣ����ü�2;
 *
 * �� ��:  ��
**********************************************************************************************************/
void  Modbus_AddCRC16(uint8_t *_pcData, uint16_t _usLen)
{
    uint8_t  ucCRCHi = 0xFF;                        // CRCֵ��λ
    uint8_t  ucCRCLo = 0xFF;                        // CRCֵ��λ
    uint16_t usIndex;                               // ����
                                                    
    while (_usLen--)                                // ��ʼ���ֽڲ��
    {
        usIndex = ucCRCHi ^ *_pcData++;             // ע�⣬����ָ���ַ��1��
        ucCRCHi = ucCRCLo ^ aModbusCRC16[usIndex];
        ucCRCLo = aModbusCRC16[usIndex + 256];
    }
    _pcData[0] = ucCRCHi;                           // ĩβ��1�ֽڣ�׷��CRC16�ĸ�λ
    _pcData[1] = ucCRCLo;                           // ĩβ��2�ֽڣ�׷��CRC16�ĵ�λ
}



/**********************************************************************************************************
 * �� ��:  Modbus_CheckCRC16
 * �� ��:  �Դ�ModbusCRC16У������ݶν���У��
 *
 * �� ��:  uint8_t*  _pcData�����ݶε�ַ
 *         uint16_t  _usLen �����ݶεĳ���(�ֽ���); �������������ݵ��ֽ�����������ModbusCRC16ֵ�ĳ��ȣ����ü�2;
 *
 * �� ��:  0-����1-��ȷ
*******************************************************************************************/
uint8_t  Modbus_CheckCRC16(uint8_t *_pcData, uint16_t _usLen)
{
    uint8_t  ucCRCHi = 0xFF;                               // CRCֵ��λ
    uint8_t  ucCRCLo = 0xFF;                               // CRCֵ��λ
    uint16_t usIndex;                                      // ���������
   
    _usLen -= 2;                                           // �ֽ���-2��������ԭ����ĩβ���ֽ�(ModbusCRC16ֵ)
    
    while (_usLen--)                                       // ��ʼ���ֽڲ����ModbusCRC16ֵ
    {
        usIndex = ucCRCHi ^ *_pcData++;                    // ע�⣬����ָ���ַ������1
        ucCRCHi = ucCRCLo ^ aModbusCRC16[usIndex];
        ucCRCLo = aModbusCRC16[usIndex + 256];
    }
    
    if ((ucCRCHi == *_pcData++) && (ucCRCLo == *_pcData))  // �����ݶε�CRC16У��ֵ����ԭ����ĩβ��CRC16ֵ��Ƚ�
        return 1;                                          // �ɹ�ƥ��; ����: 1
    
    return 0;                                              // ����; ���أ�0
}





/////////////////////////////////////////////////////////////  ��������   /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * ��  ���� debug_showData
 * ��  �ܣ� ��printf�����͵����������ϣ�����۲�
 * ��  ���� uint8_t  *rxData   ���ݵ�ַ
 *          uint16_t  rxNum    �ֽ���
 * ����ֵ�� ��
 ******************************************************************************/
#if 0
void debug_showData(uint8_t *puRxData, uint16_t usRxNum)
{
    printf("�ֽ����� %d \r", usRxNum);                   // ��ʾ�ֽ���
    printf("ASCII ��ʾ����: %s\r", (char *)puRxData);    // ��ʾ���ݣ���ASCII��ʽ��ʾ�������ַ����ķ�ʽ��ʾ
    printf("16������ʾ����: ");                          // ��ʾ���ݣ���16���Ʒ�ʽ����ʾÿһ���ֽڵ�ֵ
    while (usRxNum--)                                    // ����ֽ��жϣ�ֻҪ��Ϊ'\0', �ͼ���
        printf("0x%X ", *puRxData++);                    // ��ʽ��
    printf("\r\r");                                      // ��ʾ����
}
#endif




//////////////////////////////////////////////////////////////  printf   //////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * ����˵���� printf����֧�ִ���
 *           ���ر�ע�⡿�������´���, ʹ��printf����ʱ, ������Ҫ��use MicroLIB
 * ��    ע��
 * �����£� 2024��06��07��
 ******************************************************************************/
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};                                         // ��׼����Ҫ��֧�ֺ���

FILE __stdout;                             // FILE ��stdio.h�ļ�
void _sys_exit(int x)
{
    x = x;                                 // ����_sys_exit()�Ա���ʹ�ð�����ģʽ
}

int fputc(int ch, FILE *f)                 // �ض���fputc������ʹprintf���������fputc�����UART
{
    UART1_SendData((uint8_t *)&ch, 1);     // ʹ�ö���+�жϷ�ʽ��������; ������ʽ1�����ȴ���ʱ����Ҫ������д�õĺ��������λ���
    return ch;
}

