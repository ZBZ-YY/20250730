/***********************************************************************************************************************************
 ** ���ļ����ơ�  system_f103.c
 ** ����д��Ա��  ħŮ�������Ŷ�
 ** �����·���  QȺ�ļ���
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** ��ʹ��˵����  1- ƽ̨�޹�
 **               2- ȫ�ֺ�����
 **                  vScheduler_TickCnt()   ������ѯֵ��������������SysTick�жϷ������У����ã�������ѯ�ж�ֵ;
 **                  vScheduler_Run()       ������ѯ; while�в��ϵ��ñ���������ִ������;
 **               3- ��ֲ������
 **                  ��SysTick�жϷ��������ļ��У�#include "scheduler.h"����ʹ��ɵ��ñ��ļ�����
 **                  vScheduler_TickCnt()�ŵ�SysTick�жϷ������У����Systick����1ms�ж�һ�Σ���Ҫ�������޸�;
 **                  vScheduler_Run()�ŵ�main��whileѭ���У�
 **                  Ҫ���ִ�еĴ��룬����ŵ���vTask_xxms()�����У�
 **                  ע�⣺ÿ��vTask_xxms()��������������ʱ������Ҫ��������������ѯʱ������ʹ��System_TestRunTimes()��������ʱ��
 **
 ** �����¼�¼��  2020-04-21  ����
 **               2021-02-25  ����ע��
 **
==================================================================================================================================*/
#include "scheduler.h"



/*****************************************************************************
 ** ���ر��� ��������
 *****************************************************************************/
static uint64_t sysTickCnt = 0;      // ����ʱ������λ��ms

static u16 usTick_1ms      = 0;
static u16 usTick_2ms      = 0;
static u16 usTick_5ms      = 0;
static u16 usTick_10ms     = 0;
static u16 usTick_50ms     = 0;
static u16 usTick_100ms    = 0;
static u16 usTick_500ms    = 0;
static u16 usTick_1000ms   = 0;



/*****************************************************************************
 ** ���غ��� ��������
 *****************************************************************************/
void Scheduler_TickCnt(void); 

void vTask_1ms(void);
void vTask_2ms(void);
void vTask_5ms(void);
void vTask_10ms(void);
void vTask_50ms(void);
void vTask_100ms(void);
void vTask_500ms(void);
void vTask_1000ms(void);





/*****************************************************************************
 * ��  ���� Scheduler_Init
 * ��  �ܣ� ����systick��ʱ���� 1ms�ж�һ�Σ� ���������������System_DelayMS()��System_DelayUS()
 * ��  ����
 * ����ֵ�� 
 * ��  Ҫ�� SysTick ��ʱ��Դ�� HCLK �� 8 ��Ƶ
*****************************************************************************/
void Scheduler_Init(void)
{       
    SystemCoreClock = 5201314;             // ���ڴ��ϵͳʱ��Ƶ�ʣ���������ֵ
    SystemCoreClockUpdate();               // ��ȡ��ǰʱ��Ƶ�ʣ� ����ȫ�ֱ��� SystemCoreClockֵ 
    //printf("ϵͳ����ʱ��          %d Hz\r", SystemCoreClock);  // ϵͳʱ��Ƶ����Ϣ , SystemCoreClock��system_stm32f4xx.c�ж���   
    
    u32 msTick= SystemCoreClock /1000;     // ��������ֵ��ȫ�ֱ���SystemCoreClock��ֵ �� ������system_stm32f10x.c    
    SysTick -> LOAD  = msTick -1;          // �Զ�����
    SysTick -> VAL   = 0;                  // ��ռ�����
    SysTick -> CTRL  = 0;                  // ��0
    SysTick -> CTRL |= 1<<2;               // 0: ʱ��=HCLK/8, 1:ʱ��=HCLK
    SysTick -> CTRL |= 1<<1;               // ʹ���ж�
    SysTick -> CTRL |= 1<<0;               // ʹ��SysTick    
    
    printf("SysTickʱ������          1ms�ж�1��\r");
} 



/*****************************************************************************
 * ��  ����SysTick_Handler
 * ��  �ܣ�SysTick�жϺ���������ע�͵�stm32f10x_it.c�е�SysTick_Handler()
 * ��  ����
 * ����ֵ��
*****************************************************************************/
void SysTick_Handler(void)
{
    sysTickCnt++;          // ����DelayMS��DelayUS��ʱ��������������޹�; 1ms ��1��    
        
    Scheduler_TickCnt();   // �����������ʱ����
 
}



/*****************************************************************************
 * ��  ���� Scheduler_GetTimeMs
 * ��  �ܣ� ��ȡ��ǰ������ʱ�䣬��λ������
 * ��  ����
 * ����ֵ�� 
 * ��  ע�� �������ܺ�����������������Ǳ���
*****************************************************************************/
uint64_t Scheduler_GetTimeMs(void)
{    
    return sysTickCnt  ;
}



/*****************************************************************************
 * ��  ���� Scheduler_GetTimeUs
 * ��  �ܣ� ��ȡϵͳ�ϵ������ʱ������us
 * ��  ����
 * ����ֵ�� u32 us
 * ��  ע�� �������ܺ�����������������Ǳ���
*****************************************************************************/
u32 Scheduler_GetTimeUs(void)
{
    u32 ms;
    u32 us;
    do{
        ms = Scheduler_GetTimeMs() ;
        us = (float)ms *1000 + (SysTick ->LOAD - SysTick ->VAL )*1000/SysTick->LOAD ;
    }while(ms != Scheduler_GetTimeMs() );
    return us;        
}



/*****************************************************************************
 * ��  ���� DelayMS
 * ��  �ܣ� ������ʱ
 * ʹ  �ã� �ڵ���Scheduler_Init()�����󣬼���ʹ�� 
 * ��  ���� uint32_t ms : ��Ҫ��ʱ�ĺ�����
 * ����ֵ�� 
 * ��  ע�� �������ܺ�����������������Ǳ���
*****************************************************************************/
void DelayMS(uint32_t ms)
{    
    static uint64_t _startTime=0;
    
    _startTime = Scheduler_GetTimeMs() ;
    while( Scheduler_GetTimeMs() - _startTime < ms );            
} 



/*****************************************************************************
 * ��  ���� DelayUS
 * ��  �ܣ� ΢����ʱ; 
 * ʹ  �ã� �ڵ���Scheduler_Initt()�����󣬼���ʹ�� 
 * ��  ���� uint32_t us : ��Ҫ��ʱ��΢����
 * ����ֵ��
 * ��  ע�� �������ܺ�����������������Ǳ���
*****************************************************************************/
void DelayUS(uint32_t us)
{
    static uint64_t nowUs;
    nowUs = Scheduler_GetTimeUs ();
    while(Scheduler_GetTimeUs() - nowUs < us);    
}



/*****************************************************************************
 * ��  ���� Scheduler_GetTimeInterval
 * ��  �ܣ� ��ȡʱ���������ڲ��Դ���Ƭ������ʱ��
 * ��  ���� 
 * ����ֵ�� ��һ�ε��÷���0, ֮��ÿ�ε��÷������ϴε��õļ��ʱ��(��λ:us)
 * ��  ע�� �������ܺ�����������������Ǳ���
 *****************************************************************************/ 
u32 Scheduler_GetTimeInterval(void)
{
    static u32  lastTime=0, nowTime=0;
    
    lastTime = nowTime ;        
    nowTime = Scheduler_GetTimeUs ();               
    
    if(lastTime !=0 )                      // ���ǵ�һ�ε��� 
        return (nowTime-lastTime) ;          
   
    return 0;                              // ��1�ε���   
}



/*****************************************************************************
 * ��  ���� Scheduler_TestRunTimes
 * ��  �ܣ� ����ʱʹ�ã���ȡ����ε�����ʱ��������������� 
 * ��  ���� 
 * ����ֵ��
 * ��  ע�� �������ܺ�����������������Ǳ���
 *****************************************************************************/ 
void Scheduler_TestRunTimes(void)
{
    static u8 CNT=0;
    
    u32 intervalTimes = Scheduler_GetTimeInterval ();
    if(intervalTimes == 0)
    {
        printf("������ʱ��-����ԭ����á�\r");
        CNT++;
        Scheduler_GetTimeInterval ();
        return;
    }
    
    printf("������ʱ��-����-%d:%9u us��\r", CNT++, intervalTimes);
    Scheduler_GetTimeInterval ();
} 



/*============================================================================
 * ��  ����vScheduler_TickCnt
 * ��  �ܣ�������ѯֵ����; ����������SysTick�жϷ������У����ã�������ѯ�ж�ֵ
 * ��  ����
 * ����ֵ��
 * ��  ע�� ħŮ�������Ŷ� 2020��04��21��
============================================================================*/
void Scheduler_TickCnt(void)
{
    ++usTick_1ms;
    ++usTick_2ms;
    ++usTick_5ms;
    ++usTick_10ms;
    ++usTick_50ms;
    ++usTick_100ms;
    ++usTick_500ms;
    ++usTick_1000ms;
}



/**============================================================================
 ** ��  ����vScheduler_Run
 ** ��  �ܣ�������ѯ; while�в��ϵ��ñ���������ִ������
 ** ��  ����
 ** ����ֵ��
 ** ��  ע��ħŮ�������Ŷ� 2020��04��21��
=============================================================================*/
void Scheduler_Run(void)
{
    if (usTick_1ms   >= 1)
    {
        usTick_1ms = 0;         // ÿ1ms   ִ��1�ε�����
        vTask_1ms();
    }
    if (usTick_2ms   >= 2)
    {
        usTick_2ms = 0;         // ÿ2ms   ִ��1�ε�����
        vTask_2ms();
    }
    if (usTick_5ms   >= 5)
    {
        usTick_5ms = 0;         // ÿ5ms   ִ��1�ε�����
        vTask_5ms();
    }
    if (usTick_10ms  >= 10)
    {
        usTick_10ms = 0;        // ÿ10ms  ִ��1�ε�����
        vTask_10ms();
    }
    if (usTick_50ms  >= 50)
    {
        usTick_50ms = 0;        // ÿ50ms  ִ��1�ε�����
        vTask_50ms();
    }
    if (usTick_100ms >= 100)
    {
        usTick_100ms = 0;       // ÿ100ms ִ��1�ε�����
        vTask_100ms();
    }
    if (usTick_500ms >= 500)
    {
        usTick_500ms = 0;       // ÿ500ms ִ��1�ε�����
        vTask_500ms();
    }
    if (usTick_1000ms >= 1000)
    {
        usTick_1000ms = 0;      // ÿ1000msִ��1�� ������
        vTask_1000ms();
    }
}



/*****************************************************************************
 ** 1ms
 ** ÿ���1ms����vScheduler_Run()����һ��
 ****************************************************************************/
void vTask_1ms(void)
{
    // ÿ1msִ��һ�εĴ���

    /** ��鴥�����Ƿ��� **/
    if (XPT2046_IsPressed())                                         // ��鴥�����Ƿ���; ���أ�0-δ���¡�1-����
    {
        LCD_DrawPoint(XPT2046_GetX(), XPT2046_GetY(), GREEN);        // �ڰ��µ�λ�ã�����; ���ڲ���
        static char str[20] = {0};                                   // �½����飬���ڴ�������ַ���
        sprintf(str, "%3d  %3d", XPT2046_GetX(), XPT2046_GetY());    // ��ʽ�������ַ���
        LCD_String(185, 290, str, 12, BLACK, WHITE);                 // ��ʾ�����ַ���
    }
     
  
}



/*****************************************************************************
 ** 2ms
 ** ÿ���2ms����vScheduler_Run()����һ��
 ****************************************************************************/
void vTask_2ms(void)
{



}



/*****************************************************************************
 ** 5ms
 ** ÿ���5ms����vScheduler_Run()����һ��
 ****************************************************************************/
void vTask_5ms(void)
{


}



/*****************************************************************************
 ** 10ms
 ** ÿ���10ms����vScheduler_Run()����һ��
 ****************************************************************************/
void vTask_10ms(void)
{



}

/*****************************************************************************
 ** 50ms
 ** ÿ���50ms����vScheduler_Run()����һ��
 ****************************************************************************/
void vTask_50ms(void)
{

}



/*****************************************************************************
 ** 100ms
 ** ÿ���100ms����vScheduler_Run()����һ��
 ****************************************************************************/
void vTask_100ms(void)
{
    static char strRS485[100];
    static uint8_t timeRS485 = 0;      // ����RS485��LCD��ʾ������ʱ
    static uint8_t lcdRS485 = 0;

    if (UART1_GetRxNum())                                      // ���USART1�յ�����(�Ӵ�����λ���з������ģ�
    {                                                            
        printf("\r<<<<<< USART1 ���յ������� <<<<<<");         // ׼�����
        printf("\r ����(�ֽ���)��%d", UART1_GetRxNum());       // ����; ��λ:�ֽ�
        printf("\r ����(16����)��");                              // 16���Ʒ�ʽ��ʾ���ݣ�����۲���ʵ����
        for (uint16_t i = 0; i < UART1_GetRxNum(); i++)        // ���ÿһ�ֽ���ֵ
            printf("0x%X ", UART1_GetRxData()[i]);            
        printf("\r ����(ASCII) ��%s\r", UART1_GetRxData());    // ASCII��ʽ��ʾ���ݣ�����۲��ַ�������;

        // 1_��ӡ��������λ��
        printf("\r>>>>>> ׼�������������� >>>>>>\r");
        printf("%s", (char *)UART1_GetRxData());

        // 4_ͨ��UART3����, ������ñ������RS485\ESP8266
        UART3_SendData(UART1_GetRxData(), UART1_GetRxNum());
        printf("\r������� USART3 \r");
        
        // 3_ͨ��CAN����
        static CanTxMsg xCAN_TX;
        xCAN_TX.IDE    = CAN_Id_Extended;                       // ֡��ʽ
        xCAN_TX.ExtId  = 0xBB;                                  // ��ʶ��
        xCAN_TX.RTR    = CAN_RTR_Data;                          // ֡����
        xCAN_TX.DLC    = 8;                                     // Ҫ���͵����ݳ���, ע�⣺CAN��֡��Ч�������ֵ��8�ֽڣ�
        for (uint8_t i = 0; i < 8; i++)                         // ����Ҫ���͵����ݣ���䵽�ṹ����
            xCAN_TX.Data[i] = UART1_GetRxData()[i];             
        CAN_Transmit(CAN1, &xCAN_TX);                           // ����CAN����                                
        printf("\r������� CAN \r");                            // ��ʾ
                                                                
        // 3_����У׼������
        if (strstr((char *)UART1_GetRxData(), "XPT2046"))
        {
            XPT2046_ReCalibration();
        }
        UART1_ClearRx();                                        // ��Ҫ�������������ˣ��ѽ��������声
    }

    /******  ����CAN���� ******/
    CAN_CheckReceived();                                        // ���CAN�Ƿ��յ�������

    /******  ����USART3���� ******/
    if (UART3_GetRxNum())                                       // ���USART3�Ƿ��յ�����
    {
        printf("\r<<<<<< USART3 ���յ������� <<<<<<");          // ׼�����
        printf("\r ����(�ֽ���)��%d", UART3_GetRxNum());        // ����; ��λ:�ֽ�
        printf("\r ����(16����)��");                            // 16���Ʒ�ʽ��ʾ���ݣ�����۲���ʵ����
        for (uint16_t i = 0; i < UART3_GetRxNum(); i++)         // ���ÿһ�ֽ���ֵ
            printf("0x%X ", UART3_GetRxData()[i]);
        printf("\r ����(ASCII) ��%s\r", UART3_GetRxData());     // ��ʾ���ݣ� �����������λ��

        UART3_ClearRx();                                        // ������󣬰�RS485���õ�UART4���յ����ݱ�־��0���Է�����һ֡���ݵĽ���

        if (strstr((char *)UART3_GetRxData(), "RS485_Test"))    // �ж��Ƿ���Թ��߷�����������
        {
            UART3_SendString("RS485_OK");                       // ����һ�α��ģ�����Է�����;
        }

        timeRS485 = 0;
        lcdRS485 = 1;
        memset(strRS485, 0, 100);
        LCD_Fill(120, 226, 239, 286, BLACK);
        for (uint8_t i = 0; i < 14; i++)
            strRS485[i] = UART3_GetRxData()[i];
        LCD_String(125, 229, strRS485, 16, GREEN, BLACK);        // ASCII �ַ���
        uint8_t x = 125;
        for (uint8_t i = 0; i < 6; i++)                          // 16���� ��һ��
        {
            sprintf(strRS485, "%02X ",  UART3_GetRxData()[i]);
            LCD_String(x, 248, strRS485, 12, WHITE, BLACK);
            x = x + 18;
        }
        x = 125;
        for (uint8_t i = 6; i < 12; i++)                         // 16���� ��һ��
        {
            sprintf(strRS485, "%02X ",  UART3_GetRxData()[i]);
            LCD_String(x, 261, strRS485, 12, WHITE, BLACK);
            x = x + 18;
        }
        x = 125;
        for (uint8_t i = 12; i < 18; i++)                        // 16���� ��һ��
        {
            sprintf(strRS485, "%02X ",  UART3_GetRxData()[i]);
            LCD_String(x, 274, strRS485, 12, WHITE, BLACK);
            x = x + 18;
        }
    }
    else
    {
        if (++timeRS485 > 50 && lcdRS485 == 1)                    // �������û�յ������ݣ����LCD�ϵ���ʾ����
        {
            LCD_Fill(120, 226, 239, 286, BLACK);
            lcdRS485 = 0;
        }
    }

}



/*****************************************************************************
 ** 500ms
 ** ÿ���500ms����vScheduler_Run()����һ��
 ****************************************************************************/
void vTask_500ms(void)
{
    LED_BLUE_TOGGLE;             // ��ɫLED ÿ0.5������һ�Σ��Լ��ϵͳ��������
}



/*****************************************************************************
 ** 1000ms
 ** ÿ���1000ms����vScheduler_Run()����һ��
 ****************************************************************************/
void vTask_1000ms(void)
{

    // ʾ������ȡоƬ�ڲ��¶ȣ�����ʾ��LCD
//    static float temp ;
//    static char  strTem[20];
//    temp = ADC1_GetInteriorTemperature();
//    sprintf(strTem, "%4.1f��", temp);
//    LCD_String(66, 290, strTem, 12, BLACK, WHITE);

}


