#include "bsp_CAN.h"
#include "string.h"





/******************************************************************************
 * 函  数： CAN1_Config
 * 功  能： 配置CAN
 * 参  数： 无
 * 返回值： 无
 * 备  注： 
******************************************************************************/
void CAN1_Config(void)
{
    // 1-声明配置CAN所需的结构体
    GPIO_InitTypeDef GPIO_InitStructure;                          // GPIO配置结构体
    CAN_InitTypeDef  CAN_InitStructure;                           // CAN配置结构体
    CAN_FilterInitTypeDef CAN_FilterInitTypeStruct;               // 筛选器配置结构体
   
    // 2-使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);          // 使能CAN1时钟

#if CAN_GPIO_REMAP                                                // 此值在bsp_CAN.h文件中定义，用于判断使用PA11+PA12，或者使用PB8+PB9
    // 2-使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);         // 使能GPIOA时钟
    // 3-初始化CAN所需的GPIO:CAN_TX_PB9
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 ;                  // CAN_TX_PB9
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                 // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;            // 引脚速度
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;             //
    GPIO_Init(GPIOB, &GPIO_InitStructure);                        // 初始化引脚
    // 4-初始化CAN所需的GPIO:CAN_RX_PB8
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 ;                  // CAN_RX_PB8
    GPIO_Init(GPIOB, &GPIO_InitStructure);                        // 初始化引脚
    // 5-配置引脚复用功能
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
#else
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);         // 使能GPIOA时钟
    // 3-初始化CAN所需的GPIO:CAN_TX_PA12
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 ;                 // CAN_TX_PA12
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                 // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;             // 引脚速度
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                 //
    GPIO_Init(GPIOA, &GPIO_InitStructure);                        // 初始化引脚
    // 4-初始化CAN所需的GPIO:CAN_RX_PA11
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11 ;                 // CAN_RX_PA11
    GPIO_Init(GPIOA, &GPIO_InitStructure);                        // 初始化引脚
    // 5-配置引脚复用功能
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
#endif
  
    // 6-初始化CAN外设
    CAN_InitStructure.CAN_ABOM = ENABLE;                          // 退出离线管理; DISABLE-由软件控制; ENABLE-硬件自动控制
    CAN_InitStructure.CAN_AWUM = ENABLE;                          // 自动唤醒管理; DISABLE-由软件唤醒; ENABLE-硬件自动唤醒
    CAN_InitStructure.CAN_NART = ENABLE;                          // 禁止自动重传; DISABLE-发送报文失败时一直自动重传直到发送成功; ENABLE-报文只被发送1次，不管发送的结果如何
    CAN_InitStructure.CAN_RFLM = DISABLE;                         // 接收FIFO锁定; DISABLE-当接收溢出时，FIFO的报文未被读出，新到的报文将覆盖旧报文; ENABLE-新报文不能覆盖旧报文，直接舍弃
    CAN_InitStructure.CAN_TTCM = DISABLE;                         // 时间触发模式：DISABLE-禁止时间触发通信模式; ENABLE-允许时间触发通信模式
    CAN_InitStructure.CAN_TXFP = ENABLE;                          // 发送的优先级; DISABLE-发送优先级由报文的标识符来决定; ENABLE-发送优先级按报文存入邮箱的先后顺序发送
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal ;                // 工作模式，　Normal=正常, LoopBack=回环, Silent=静默, Silent_LoopBack=静默回环
 
    // 7-波特率、位时间配置_500K
    CAN_InitStructure.CAN_Prescaler = 6;                          // 分频系数, 直接填写要的分频值，函数会自动减1
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;                      // 时间段1
    CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;                     // 时间段2
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;                      // 时间段3
    CAN_Init(CAN1, &CAN_InitStructure);                           // 初始化CAN控制器，即把上述参数写入寄存器中
    
    // 8-配置过滤器，规则：接收所有数据帧
    CAN_FilterInitTypeStruct.CAN_FilterNumber = 1;                                                                       // 指定使用哪个过滤器; F103系列可选:0~13, F105和F107系列可选:0~27
    CAN_FilterInitTypeStruct.CAN_FilterMode   = CAN_FilterMode_IdMask ;                                                  // 过滤模式；IDMask=0=屏蔽位模式；IdList=1=列表模式
    CAN_FilterInitTypeStruct.CAN_FilterScale  = CAN_FilterScale_32bit ;                                                  // 过滤器位宽
    CAN_FilterInitTypeStruct.CAN_FilterIdHigh = (((uint32_t)0x000 << 3) & 0xFFFF0000) >> 16;                             // 掩码模式时，验证码的高16位值
    CAN_FilterInitTypeStruct.CAN_FilterIdLow  = (((uint32_t)0x000 << 3 | CAN_Id_Standard | CAN_RTR_Data) & 0xFFFF);      // 掩码模式时，验证码的低16位值
    CAN_FilterInitTypeStruct.CAN_FilterMaskIdHigh = ((uint32_t)0x000 << 3) & 0xFFFF0000 >> 16;                           // 过滤器屏蔽标识符的高16位; 重点建议，用二进制来看待此值，否则很难理解; 当设置相关位为0时，表示不用管总线上的帧ID这个位是啥值，都通过; 当位为1时，帧ID的这个位，必须与验证码此位的值相同，才能通过
    CAN_FilterInitTypeStruct.CAN_FilterMaskIdLow  = (((uint32_t)0x000 << 3 | CAN_Id_Standard | CAN_RTR_Data) & 0xFFFF);  // 过滤器屏蔽标识符的低16位; 接上，如果高低位寄存器，都设置为0x0000，表示，不用与验证码对比，所有帧ID都通过，接收所有报文数据; 如果都设置为0xFFFF, 表示帧ID值，每一位都与验证码相同，才接收报文;
    CAN_FilterInitTypeStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;                                                // FIFO关联；上面这个过滤器，接收到的数据，存放到FIFO0; CAN共有两个接收FIFO，每个FIFO包含3个接收邮箱，都是完全由硬件管理; 接收到的报文，会自动存放到邮箱中,读取时，自动读出最先收到的报文,不用管是哪个邮箱的)
    CAN_FilterInitTypeStruct.CAN_FilterActivation = ENABLE;                                                              // 使能过滤器
    CAN_FilterInit(&CAN_FilterInitTypeStruct);
    
    printf("CAN1 初始化配置          配置完成;接收监听中...\r");
}
    


// 发送CAN报文
void CAN_SendData(void)
{
    // 准备CAN要发送的数据
    static char strTest[8] = "CAN_Test";
    // 配置发送的帧类型
    static CanTxMsg xCAN_TX;
    xCAN_TX.IDE    = CAN_Id_Standard;                                           // 帧格式
    xCAN_TX.ExtId  = 0xBB;                                                      // 标识符
    xCAN_TX.RTR    = CAN_RTR_Data;                                              // 帧类型
    xCAN_TX.DLC    = 8;                                                         // 要发送的数据长度, 注意：CAN最帧有效数据最大值，8字节！
    for (uint8_t i = 0; i < 8; i++)                                             // 把需要发送的数据，填充到结构体中
        xCAN_TX.Data[i] = strTest[i];
    // 发送
    CAN_Transmit(CAN1, &xCAN_TX);                                               // 发送CAN报文
}



// 检查是否接收到了新报文; 如果是，即打印到串口软件，方便观察
void CAN_CheckReceived(void)
{
    static uint16_t  canReceivedCNT = 1;                                        // 用于计算已接收了多少帧数据; 非必要;
    static CanRxMsg  xCAN_RX;
    static char      canString[9];
     
    static char strCAN[100];
    static uint16_t timeCAN = 0;        // 用于CAN的LCD显示请屏计时
    static uint8_t lcdCAN = 0;

    if (CAN_MessagePending(CAN1, CAN_FIFO0))                                    // 如果收到了CAN报文
    {
        memset((void *)&xCAN_RX, 0x00, sizeof(xCAN_RX));                        // 结构体数据先清0
        CAN_Receive(CAN1, CAN_FIFO0, &xCAN_RX);                                 // 把接收到的CAN报文，解释并存入到结构体中

        printf("\r****** CAN 接收到第%d帧新数据 ******", canReceivedCNT++);     // 准备把CAN帧报文，详细地输出到串口软件，方便观察调试
        printf("\r 帧类型：%s",   xCAN_RX.RTR ? "遥控帧" : "数据帧");           // 帧类型
        printf("\r 帧格式：%s",   xCAN_RX.IDE ? "扩展帧" : "标准帧");           // 帧格式
        printf("\r 标识符：0x%X", xCAN_RX.IDE ? xCAN_RX.ExtId : xCAN_RX.StdId); // 扩展帧ID
        printf("\r 字节数：%d",   xCAN_RX.DLC);                                 // 字节数
        printf("\r 过滤器匹配序号：%d", xCAN_RX.FMI);                           // 过滤器匹配序号; 和过滤器编号，是不一样的。大概：从过滤器0开始，每个16位宽过滤器+2, 32位宽+1, 没有被使用的过滤器，默认是16位宽，+2;

        printf("\r 显示数据(16进制)：");                                        // 16进制方式显示数据，方便观察真实数据
        for (uint8_t i = 0; i < 8; i++)
            printf(" 0x%X ", xCAN_RX.Data[i]);

        memcpy(canString, xCAN_RX.Data, 8);
        printf("\r 显示数据(ASCII) ： %s\r", canString);                        // ASCII方式显示数据，方便观察字符串数据; xCAN.ReceivedBuf[9]所开辟9个字节，就是为了方便这种方式输出，第9字节为:'\0'

        if (strstr((char *)xCAN_RX.Data, "CAN_Test"))                           // 判断是否测试工具发过来的数据
        {
            static char strTemp[8] = "CAN_OK";                                  // 发回指定数据
            static CanTxMsg xCAN_TX;
            xCAN_TX.IDE    = CAN_Id_Extended;                                   // 帧格式
            xCAN_TX.ExtId  = 0xBB;                                              // 标识符
            xCAN_TX.RTR    = CAN_RTR_Data;                                      // 帧类型
            xCAN_TX.DLC    = 8;                                                 // 要发送的数据长度, 注意：CAN最帧有效数据最大值，8字节！
            for (uint8_t i = 0; i < 8; i++)                                     // 把需要发送的数据，填充到结构体中
                xCAN_TX.Data[i] = strTemp[i];
            CAN_Transmit(CAN1, &xCAN_TX);                                       // 发送CAN报文
        }           

        timeCAN = 0;
        lcdCAN = 1;
        LCD_Fill(120, 146, 239, 204, BLACK);                                    // 把数据显示到LCD上方便观察
        LCD_String(125, 149, (char *)canString, 16, GREEN, BLACK);
        sprintf(strCAN, "%X %X %X %X", xCAN_RX.Data[0], xCAN_RX.Data[1], xCAN_RX.Data[2], xCAN_RX.Data[3]);
        LCD_String(125, 168, strCAN, 16, WHITE, BLACK);
        sprintf(strCAN, "%X %X %X %X", xCAN_RX.Data[4], xCAN_RX.Data[5], xCAN_RX.Data[6], xCAN_RX.Data[7]);
        LCD_String(125, 185, strCAN, 16, WHITE, BLACK);
    }
    else
    {
        if (++timeCAN > 5000 && lcdCAN == 1)                                    // 超过N秒没收到新数据，清空LCD上的显示数据
        {
            LCD_Fill(120, 146, 239, 204, BLACK);
            lcdCAN = 0;
        }
    }    
}


