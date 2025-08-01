/***********************************************************************************************************************************
 ** 【文件名称】  system_f103.c
 ** 【编写人员】  魔女开发板团队
 ** 【更新分享】  Q群文件夹
 ** 【淘    宝】  魔女开发板      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【使用说明】  1- 平台无关
 **               2- 全局函数：
 **                  vScheduler_TickCnt()   任务轮询值处理；本函数插在SysTick中断服务函数中，作用：任务轮询判断值;
 **                  vScheduler_Run()       任务轮询; while中不断调用本函数，以执行任务;
 **               3- 移植方法：
 **                  在SysTick中断服务函数的文件中：#include "scheduler.h"，以使其可调用本文件函数
 **                  vScheduler_TickCnt()放到SysTick中断服务函数中，如果Systick不是1ms中断一次，则要作代码修改;
 **                  vScheduler_Run()放到main的while循环中；
 **                  要间隔执行的代码，按需放到各vTask_xxms()函数中；
 **                  注意：每个vTask_xxms()任务函数的总运行时长，不要超过本函数的轮询时长；可使用System_TestRunTimes()测试运行时长
 **
 ** 【更新记录】  2020-04-21  创建
 **               2021-02-25  完善注释
 **
==================================================================================================================================*/
#include "scheduler.h"



/*****************************************************************************
 ** 本地变量 声明定义
 *****************************************************************************/
static uint64_t sysTickCnt = 0;      // 运行时长，单位：ms

static u16 usTick_1ms      = 0;
static u16 usTick_2ms      = 0;
static u16 usTick_5ms      = 0;
static u16 usTick_10ms     = 0;
static u16 usTick_50ms     = 0;
static u16 usTick_100ms    = 0;
static u16 usTick_500ms    = 0;
static u16 usTick_1000ms   = 0;



/*****************************************************************************
 ** 本地函数 声明定义
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
 * 函  数： Scheduler_Init
 * 功  能： 配置systick定时器， 1ms中断一次， 用于任务调度器、System_DelayMS()、System_DelayUS()
 * 参  数：
 * 返回值： 
 * 重  要： SysTick 的时钟源自 HCLK 的 8 分频
*****************************************************************************/
void Scheduler_Init(void)
{       
    SystemCoreClock = 5201314;             // 用于存放系统时钟频率，先随便设个值
    SystemCoreClockUpdate();               // 获取当前时钟频率， 更新全局变量 SystemCoreClock值 
    //printf("系统运行时钟          %d Hz\r", SystemCoreClock);  // 系统时钟频率信息 , SystemCoreClock在system_stm32f4xx.c中定义   
    
    u32 msTick= SystemCoreClock /1000;     // 计算重载值，全局变量SystemCoreClock的值 ， 定义在system_stm32f10x.c    
    SysTick -> LOAD  = msTick -1;          // 自动重载
    SysTick -> VAL   = 0;                  // 清空计数器
    SysTick -> CTRL  = 0;                  // 清0
    SysTick -> CTRL |= 1<<2;               // 0: 时钟=HCLK/8, 1:时钟=HCLK
    SysTick -> CTRL |= 1<<1;               // 使能中断
    SysTick -> CTRL |= 1<<0;               // 使能SysTick    
    
    printf("SysTick时钟配置          1ms中断1次\r");
} 



/*****************************************************************************
 * 函  数：SysTick_Handler
 * 功  能：SysTick中断函数，必须注释掉stm32f10x_it.c中的SysTick_Handler()
 * 参  数：
 * 返回值：
*****************************************************************************/
void SysTick_Handler(void)
{
    sysTickCnt++;          // 用于DelayMS和DelayUS计时，与任务调试器无关; 1ms 加1次    
        
    Scheduler_TickCnt();   // 任务调试器计时处理
 
}



/*****************************************************************************
 * 函  数： Scheduler_GetTimeMs
 * 功  能： 获取当前的运行时间，单位：毫秒
 * 参  数：
 * 返回值： 
 * 备  注： 辅助功能函数，对任务调试器非必须
*****************************************************************************/
uint64_t Scheduler_GetTimeMs(void)
{    
    return sysTickCnt  ;
}



/*****************************************************************************
 * 函  数： Scheduler_GetTimeUs
 * 功  能： 获取系统上电后运行时间数：us
 * 参  数：
 * 返回值： u32 us
 * 备  注： 辅助功能函数，对任务调试器非必须
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
 * 函  数： DelayMS
 * 功  能： 毫秒延时
 * 使  用： 在调用Scheduler_Init()函数后，即可使用 
 * 参  数： uint32_t ms : 需要延时的毫秒数
 * 返回值： 
 * 备  注： 辅助功能函数，对任务调试器非必须
*****************************************************************************/
void DelayMS(uint32_t ms)
{    
    static uint64_t _startTime=0;
    
    _startTime = Scheduler_GetTimeMs() ;
    while( Scheduler_GetTimeMs() - _startTime < ms );            
} 



/*****************************************************************************
 * 函  数： DelayUS
 * 功  能： 微秒延时; 
 * 使  用： 在调用Scheduler_Initt()函数后，即可使用 
 * 参  数： uint32_t us : 需要延时的微秒数
 * 返回值：
 * 备  注： 辅助功能函数，对任务调试器非必须
*****************************************************************************/
void DelayUS(uint32_t us)
{
    static uint64_t nowUs;
    nowUs = Scheduler_GetTimeUs ();
    while(Scheduler_GetTimeUs() - nowUs < us);    
}



/*****************************************************************************
 * 函  数： Scheduler_GetTimeInterval
 * 功  能： 获取时间间隔，用于测试代码片段运行时间
 * 参  数： 
 * 返回值： 第一次调用返回0, 之后每次调用返回与上次调用的间隔时间(单位:us)
 * 备  注： 辅助功能函数，对任务调试器非必须
 *****************************************************************************/ 
u32 Scheduler_GetTimeInterval(void)
{
    static u32  lastTime=0, nowTime=0;
    
    lastTime = nowTime ;        
    nowTime = Scheduler_GetTimeUs ();               
    
    if(lastTime !=0 )                      // 不是第一次调用 
        return (nowTime-lastTime) ;          
   
    return 0;                              // 第1次调用   
}



/*****************************************************************************
 * 函  数： Scheduler_TestRunTimes
 * 功  能： 调试时使用，获取代码段的运行时长，并输出到串口 
 * 参  数： 
 * 返回值：
 * 备  注： 辅助功能函数，对任务调试器非必须
 *****************************************************************************/ 
void Scheduler_TestRunTimes(void)
{
    static u8 CNT=0;
    
    u32 intervalTimes = Scheduler_GetTimeInterval ();
    if(intervalTimes == 0)
    {
        printf("【运行时长-测试原点放置】\r");
        CNT++;
        Scheduler_GetTimeInterval ();
        return;
    }
    
    printf("【运行时长-监察点-%d:%9u us】\r", CNT++, intervalTimes);
    Scheduler_GetTimeInterval ();
} 



/*============================================================================
 * 函  数：vScheduler_TickCnt
 * 功  能：任务轮询值处理; 本函数插在SysTick中断服务函数中，作用：任务轮询判断值
 * 参  数：
 * 返回值：
 * 备  注： 魔女开发板团队 2020年04月21日
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
 ** 函  数：vScheduler_Run
 ** 功  能：任务轮询; while中不断调用本函数，以执行任务
 ** 参  数：
 ** 返回值：
 ** 备  注：魔女开发板团队 2020年04月21日
=============================================================================*/
void Scheduler_Run(void)
{
    if (usTick_1ms   >= 1)
    {
        usTick_1ms = 0;         // 每1ms   执行1次的任务
        vTask_1ms();
    }
    if (usTick_2ms   >= 2)
    {
        usTick_2ms = 0;         // 每2ms   执行1次的任务
        vTask_2ms();
    }
    if (usTick_5ms   >= 5)
    {
        usTick_5ms = 0;         // 每5ms   执行1次的任务
        vTask_5ms();
    }
    if (usTick_10ms  >= 10)
    {
        usTick_10ms = 0;        // 每10ms  执行1次的任务
        vTask_10ms();
    }
    if (usTick_50ms  >= 50)
    {
        usTick_50ms = 0;        // 每50ms  执行1次的任务
        vTask_50ms();
    }
    if (usTick_100ms >= 100)
    {
        usTick_100ms = 0;       // 每100ms 执行1次的任务
        vTask_100ms();
    }
    if (usTick_500ms >= 500)
    {
        usTick_500ms = 0;       // 每500ms 执行1次的任务
        vTask_500ms();
    }
    if (usTick_1000ms >= 1000)
    {
        usTick_1000ms = 0;      // 每1000ms执行1次 的任务
        vTask_1000ms();
    }
}



/*****************************************************************************
 ** 1ms
 ** 每间隔1ms，被vScheduler_Run()调用一次
 ****************************************************************************/
void vTask_1ms(void)
{
    // 每1ms执行一次的代码

    /** 检查触摸屏是否按下 **/
    if (XPT2046_IsPressed())                                         // 检查触摸屏是否按下; 返回：0-未按下、1-按下
    {
        LCD_DrawPoint(XPT2046_GetX(), XPT2046_GetY(), GREEN);        // 在按下的位置，画点; 用于测试
        static char str[20] = {0};                                   // 新建数组，用于存放坐标字符串
        sprintf(str, "%3d  %3d", XPT2046_GetX(), XPT2046_GetY());    // 格式化坐标字符串
        LCD_String(185, 290, str, 12, BLACK, WHITE);                 // 显示坐标字符串
    }
     
  
}



/*****************************************************************************
 ** 2ms
 ** 每间隔2ms，被vScheduler_Run()调用一次
 ****************************************************************************/
void vTask_2ms(void)
{



}



/*****************************************************************************
 ** 5ms
 ** 每间隔5ms，被vScheduler_Run()调用一次
 ****************************************************************************/
void vTask_5ms(void)
{


}



/*****************************************************************************
 ** 10ms
 ** 每间隔10ms，被vScheduler_Run()调用一次
 ****************************************************************************/
void vTask_10ms(void)
{



}

/*****************************************************************************
 ** 50ms
 ** 每间隔50ms，被vScheduler_Run()调用一次
 ****************************************************************************/
void vTask_50ms(void)
{

}



/*****************************************************************************
 ** 100ms
 ** 每间隔100ms，被vScheduler_Run()调用一次
 ****************************************************************************/
void vTask_100ms(void)
{
    static char strRS485[100];
    static uint8_t timeRS485 = 0;      // 用于RS485的LCD显示清屏计时
    static uint8_t lcdRS485 = 0;

    if (UART1_GetRxNum())                                      // 如果USART1收到数据(从串口上位机中发过来的）
    {                                                            
        printf("\r<<<<<< USART1 接收到新数据 <<<<<<");         // 准备输出
        printf("\r 长度(字节数)：%d", UART1_GetRxNum());       // 长度; 单位:字节
        printf("\r 数据(16进制)：");                              // 16进制方式显示数据，方便观察真实数据
        for (uint16_t i = 0; i < UART1_GetRxNum(); i++)        // 输出每一字节数值
            printf("0x%X ", UART1_GetRxData()[i]);            
        printf("\r 数据(ASCII) ：%s\r", UART1_GetRxData());    // ASCII方式显示数据，方便观察字符串数据;

        // 1_打印到串口上位机
        printf("\r>>>>>> 准备发送下面数据 >>>>>>\r");
        printf("%s", (char *)UART1_GetRxData());

        // 4_通过UART3发送, 视跳线帽而定：RS485\ESP8266
        UART3_SendData(UART1_GetRxData(), UART1_GetRxNum());
        printf("\r已输出到 USART3 \r");
        
        // 3_通过CAN发送
        static CanTxMsg xCAN_TX;
        xCAN_TX.IDE    = CAN_Id_Extended;                       // 帧格式
        xCAN_TX.ExtId  = 0xBB;                                  // 标识符
        xCAN_TX.RTR    = CAN_RTR_Data;                          // 帧类型
        xCAN_TX.DLC    = 8;                                     // 要发送的数据长度, 注意：CAN最帧有效数据最大值，8字节！
        for (uint8_t i = 0; i < 8; i++)                         // 把需要发送的数据，填充到结构体中
            xCAN_TX.Data[i] = UART1_GetRxData()[i];             
        CAN_Transmit(CAN1, &xCAN_TX);                           // 发送CAN报文                                
        printf("\r已输出到 CAN \r");                            // 提示
                                                                
        // 3_重新校准触摸屏
        if (strstr((char *)UART1_GetRxData(), "XPT2046"))
        {
            XPT2046_ReCalibration();
        }
        UART1_ClearRx();                                        // 重要：处理完数据了，把接收数量清０
    }

    /******  处理CAN报文 ******/
    CAN_CheckReceived();                                        // 检查CAN是否收到新数据

    /******  处理USART3数据 ******/
    if (UART3_GetRxNum())                                       // 检查USART3是否收到数据
    {
        printf("\r<<<<<< USART3 接收到新数据 <<<<<<");          // 准备输出
        printf("\r 长度(字节数)：%d", UART3_GetRxNum());        // 长度; 单位:字节
        printf("\r 数据(16进制)：");                            // 16进制方式显示数据，方便观察真实数据
        for (uint16_t i = 0; i < UART3_GetRxNum(); i++)         // 输出每一字节数值
            printf("0x%X ", UART3_GetRxData()[i]);
        printf("\r 数据(ASCII) ：%s\r", UART3_GetRxData());     // 显示数据， 输出到电脑上位机

        UART3_ClearRx();                                        // 处理完后，把RS485所用的UART4接收到数据标志清0，以方便下一帧数据的接收

        if (strstr((char *)UART3_GetRxData(), "RS485_Test"))    // 判断是否测试工具发过来的数据
        {
            UART3_SendString("RS485_OK");                       // 发回一次报文，方便对方测试;
        }

        timeRS485 = 0;
        lcdRS485 = 1;
        memset(strRS485, 0, 100);
        LCD_Fill(120, 226, 239, 286, BLACK);
        for (uint8_t i = 0; i < 14; i++)
            strRS485[i] = UART3_GetRxData()[i];
        LCD_String(125, 229, strRS485, 16, GREEN, BLACK);        // ASCII 字符串
        uint8_t x = 125;
        for (uint8_t i = 0; i < 6; i++)                          // 16进制 第一行
        {
            sprintf(strRS485, "%02X ",  UART3_GetRxData()[i]);
            LCD_String(x, 248, strRS485, 12, WHITE, BLACK);
            x = x + 18;
        }
        x = 125;
        for (uint8_t i = 6; i < 12; i++)                         // 16进制 第一行
        {
            sprintf(strRS485, "%02X ",  UART3_GetRxData()[i]);
            LCD_String(x, 261, strRS485, 12, WHITE, BLACK);
            x = x + 18;
        }
        x = 125;
        for (uint8_t i = 12; i < 18; i++)                        // 16进制 第一行
        {
            sprintf(strRS485, "%02X ",  UART3_GetRxData()[i]);
            LCD_String(x, 274, strRS485, 12, WHITE, BLACK);
            x = x + 18;
        }
    }
    else
    {
        if (++timeRS485 > 50 && lcdRS485 == 1)                    // 超过多久没收到新数据，清空LCD上的显示数据
        {
            LCD_Fill(120, 226, 239, 286, BLACK);
            lcdRS485 = 0;
        }
    }

}



/*****************************************************************************
 ** 500ms
 ** 每间隔500ms，被vScheduler_Run()调用一次
 ****************************************************************************/
void vTask_500ms(void)
{
    LED_BLUE_TOGGLE;             // 蓝色LED 每0.5秒闪灭一次，以监察系统正常工作
}



/*****************************************************************************
 ** 1000ms
 ** 每间隔1000ms，被vScheduler_Run()调用一次
 ****************************************************************************/
void vTask_1000ms(void)
{

    // 示例：获取芯片内部温度，并显示在LCD
//    static float temp ;
//    static char  strTem[20];
//    temp = ADC1_GetInteriorTemperature();
//    sprintf(strTem, "%4.1f℃", temp);
//    LCD_String(66, 290, strTem, 12, BLACK, WHITE);

}


