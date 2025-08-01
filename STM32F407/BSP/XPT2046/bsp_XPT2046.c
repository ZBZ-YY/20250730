#include "bsp_XPT2046.h"            // ͷ�ļ�
#include "bsp_W25Q128.h"            // W25Q128�������ļ�����������У׼���ݴ��λ�ã�0x00A0000-100�����ֿ�ǰ100�ֽ�                                    
#include "string.h"                 // C���Եı�׼�����⣬�����ַ�������
#include "stdio.h"                  // C���Եı�׼�����⣬����printf�Ⱥ���
                                    



/******************************* ������صľ�̬���� ***************************/
typedef    struct                   // ����������ṹ��
{                                   
    uint8_t EN;                     // ������⿪��, 0_�ر�, 1_��; �رպʹ�, ��ָ�����ϵļ�����, ����ʱ�ɹر�, ����оƬ��Դ����
                                    
    uint16_t lcdX;                  // ��ǰ���µ�LCD����ֵ(
    uint16_t lcdY;                  
                                    
    uint16_t adcX;                  // ��������ȡ�Ĵ�����X�����ADCֵ������ƽ��ֵ�˲�
    uint16_t adcY;                  
                                    
    uint16_t lcdWidth;              // ���ڼ�¼LCDʵ�ʵĿ������; ��XPT2046_Init�e�xֵ
    uint16_t lcdHeight;             // ���ڼ�¼LCDʵ�ʵĸ߶�����; ��XPT2046_Init�e�xֵ
                                    
    float xfac;                     // ��������LCD���������ϵ��,  xfac=(float)(20-320)/(t1x-t2x);
    float yfac;                     
    short xoff;                     // ���ص�ƫ��ֵ, xoff=(320-xfac*(t1x+t2x))/2;
    short yoff;                     
                                    
    uint8_t  dir;                   // ��ʾ����, 0-����, 1_����
    uint32_t dataAddr;              // �����������ڲ�FLASH�еĴ�ŵ�ַ
} xXPT2046_TypeDey;

static xXPT2046_TypeDey xXPT2046;   // ���ڴ��XPT2046�ĸ�����Ϣ��ȫ�ֱ�������h��extern





/******************************* �궨�� ***************************/
#define XPT2046_CHANNEL_X   0x90         // �����֣����ͨ��Y+��ѹֵ    
#define XPT2046_CHANNEL_Y   0xD0         // �����֣����ͨ��X+��ѹֵ

#define  IRQ_READ           ( XPT2046_IRQ_GPIO -> IDR & XPT2046_IRQ_PIN )                  // �����ź����ţ���ʱ�ߵ�ƽ������ʱ�͵�ƽ
#define  CS_HIGH            ( XPT2046_CS_GPIO  -> BSRR  = XPT2046_CS_PIN )
#define  CS_LOW             ( XPT2046_CS_GPIO  -> BSRR  = XPT2046_CS_PIN << 16 )
#define  CLK_HIGH           ( XPT2046_CLK_GPIO -> BSRR  = XPT2046_CLK_PIN)
#define  CLK_LOW            ( XPT2046_CLK_GPIO -> BSRR  = XPT2046_CLK_PIN << 16 )
#define  MOSI_1             ( XPT2046_MOSI_GPIO-> BSRR  = XPT2046_MOSI_PIN )
#define  MOSI_0             ( XPT2046_MOSI_GPIO-> BSRR  = XPT2046_MOSI_PIN << 16 )
#define  MISO               ( (( XPT2046_MISO_GPIO -> IDR) & XPT2046_MISO_PIN ) ? 1 : 0 )





/******************************* �������غ��� ***************************/
static void      delayUS(uint32_t ulCount);       // ���Ե���ʱ����, Ϊ������ֲ, ��ʹ���ⲿ��ʱ����
static void      sendCMD(uint8_t cmd);            // ����������
static uint16_t  receiveData(void);               // ��ȡ����ֵ
static int16_t   readADC_X(void);                 // ��ȡX��ADCֵ
static int16_t   readADC_Y(void);                 // ��ȡY��ADCֵ
static uint8_t   readAdcXY(void);                 // ��ȡX��Y��ADCֵ���˲�����ŵ�ȫ�ֽṹ�����xXPT2046��






// ����US������ʱ������������ֲʱ���ⲿ�ļ�������
static void delayUS(uint32_t us)
{
    for (uint32_t i = 0; i < us; i++)
    {
        uint8_t uc = 12;     //����ֵΪ12����Լ��1΢��
        while (uc --);       //��1΢��
    }
}



// д��������
// Cmd ��0x90_ͨ��Y+��ѡ�������, 0xd0_ͨ��X+��ѡ�������
// XPT2046ʱ��Ҫ��CLK��ʱ�͵�ƽ�������ز������ݣ��½��ظı�����
// ע�⣺���������ֺ󣬷��ص����ݣ��������֣���ͬһ��������(CS�͡��ߵ�ƽ�ڼ�)
static void sendCMD(uint8_t cmd)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        ((cmd >> (7 - i)) & 0x01) ? MOSI_1 : MOSI_0; // ��λ����
        delayUS(1);
        CLK_HIGH  ;
        delayUS(1);
        CLK_LOW   ;
    }
}



// ��ȡ��������(����sendCMD��������);
// ���ض�ȡ��������
static uint16_t receiveData(void)
{
    uint16_t usBuf = 0;

    CLK_HIGH;              // ��һ��ʱ�ӣ����BUSY�����ʱ���Ǹ��ڷ��������ֺ���ģ�ADת����Ҫ��Լ6US
    delayUS(5);
    CLK_LOW ;
    delayUS(5);

    for (uint8_t i = 0; i < 12; i++)
    {
        usBuf = usBuf << 1;
        CLK_HIGH;
        delayUS(1);
        usBuf |= MISO ;    // ��λ����
        CLK_LOW;
        delayUS(1);
    }

    return usBuf;
}



// ѡ��һ��ģ��ͨ��������ADC��������ADC�������
// 0x90 :ͨ��Y+��ѡ�������
// 0xd0 :ͨ��X+��ѡ�������
static int16_t readADC_X(void)
{
    if (xXPT2046.dir == 0)    sendCMD(XPT2046_CHANNEL_Y);
    if (xXPT2046.dir == 1)    sendCMD(XPT2046_CHANNEL_X);
    return  receiveData();
}



static int16_t readADC_Y(void)
{
    if (xXPT2046.dir == 0)    sendCMD(XPT2046_CHANNEL_X);
    if (xXPT2046.dir == 1)    sendCMD(XPT2046_CHANNEL_Y);
    return  receiveData();
}



/******************************************************************************
 * �������� readAdcXY
 * ��  �ܣ� ��ȡ����������ʱX��Y��ADCֵ�����˲�
 * ��  ����
 * ��  �أ� 1  ��ȡ�ɹ����Ѵ�ŵ�����Ľṹ����
 *          0  ʧ��
 ******************************************************************************/
static uint8_t readAdcXY()
{
    static uint8_t  cnt = 0;
    static uint16_t xSum = 0, ySum = 0;
    static int16_t  xyArray [2] [10] = {{0}, {0}};    // ��ʱ��ά���飬���ڴ������X��Y��10�β���
    int32_t  xMin, xMax, yMin, yMax;                  // �洢�����е���Сֵ�����ֵ;��������κ�ȥͷȥβ��ƽ��ֵ

    cnt = 0;
    xSum = 0;
    ySum = 0;
    memset(xyArray, 0, 20);
    xMin = 0;
    xMax = 0;
    yMin = 0;
    yMax = 0;

    while ((IRQ_READ == 0) && (cnt < 4))              // ��βɼ�; ������TP_INT_IN�ź�Ϊ��(��Ļ������), �� cnt<10
    {
        xyArray[0] [cnt] = readADC_X();
        xyArray[1] [cnt] = readADC_Y();
        cnt ++;
    }

    // ��ʼ��ƽ��ֵ
    xMax = xMin = xyArray [0] [0];                              // ɸѡ�Ȼ�Ҫȥ������Сֵ�����ֵ
    yMax = yMin = xyArray [1] [0];
    for (uint8_t i = 1; i < cnt; i++)
    {
        if (xyArray[0] [i] < xMin)    xMin = xyArray [0] [i];   // ��x��10�β�����СADCֵ
        if (xyArray[0] [i] > xMax)    xMax = xyArray [0] [i];   // ��x��10�β������ADCֵ

        if (xyArray[1] [i] < yMin)    yMin = xyArray [1] [i];   // ��y��10�β�����СADCֵ
        if (xyArray[1] [i] > yMax)    yMax = xyArray [1] [i];   // ��y��10�β�����СADCֵ
    }
    // ȥ����Сֵ�����ֵ֮����ƽ��ֵ
    for (uint8_t i = 0; i < cnt; i++)
    {
        xSum = xSum + xyArray[0][i];
        ySum = ySum + xyArray[1][i];
    }
    xXPT2046.adcX = (xSum - xMin - xMax) >> 1;  // ȥ����Сֵ�����ֵ֮�󣬳�2
    xXPT2046.adcY = (ySum - yMin - yMax) >> 1;  // ȥ����Сֵ�����ֵ֮�󣬳�2

    return 1;
}



// �ѵ�ѹֵ����ɶ�Ӧ��LCD����ֵ
static void adcXYToLcdXY(void)
{
    static int16_t lcdX = 0;
    static int16_t lcdY = 0;
    // �������ϵ��
    lcdX = xXPT2046.adcX * xXPT2046.xfac + xXPT2046.xoff ;
    lcdY = xXPT2046.adcY * xXPT2046.yfac + xXPT2046.yoff ;
    // ��������ֵ��Χ
    if (lcdX < 0)  lcdX = 0;
    if (lcdX > xXPT2046.lcdWidth)  lcdX = xXPT2046.lcdWidth;
    if (lcdY < 0)  lcdY = 0;
    if (lcdY > xXPT2046.lcdHeight)  lcdY = xXPT2046.lcdHeight;
    // ������, ����ֵ�������ֵ, ת�浽�ṹ��, ��ʱ�ɵ���
    xXPT2046.lcdX = lcdX;
    xXPT2046.lcdY = lcdY;
}



/******************************************************************************
 * �������� writeDcorrectingData
 * ��  �ܣ� д��У������
 * ��  ���� float xfac   x���������
 *          float yfac   y���������
 *          short xoff   x��ƫ������
 *          short yoff   y��ƫ������
 * ��  �أ� 0_д��ɹ�����0_д��ʧ��
 ******************************************************************************/
static uint8_t writeCorrectingData(float xfac, float yfac, short xoff, short yoff)
{
    uint32_t addr = 0;
    if (xXPT2046.dir == 0)
        addr = xXPT2046.dataAddr;
    if (xXPT2046.dir == 1)
        addr = xXPT2046.dataAddr + 20;

    uint8_t err = 0;
    uint16_t flag = 'O' | ('K' << 8);
    W25Q128_WriteData(addr + 0, (uint8_t *)&flag, 2);
    W25Q128_WriteData(addr + 2, (uint8_t *)&xfac, 4);
    W25Q128_WriteData(addr + 6, (uint8_t *)&xoff, 4);
    W25Q128_WriteData(addr + 10, (uint8_t *)&yfac, 4);
    W25Q128_WriteData(addr + 14, (uint8_t *)&yoff, 4);
    return err;
}



/******************************************************************************
 * �������� XPT2046_Init
 * ��  �ܣ� ��ʼ��
 * ��  ���� uint16_t lcdWidth     LCD�������
 *          uint16_t lcdHeight    LCD�������
 *          uint8_t dir           ��ʾ����    0-��������3-��������5-������, 6-������
 * ��  �أ�
 ******************************************************************************/
void XPT2046_Init(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    xXPT2046.dataAddr  = 0x00A00000 - 100 ;                      // У׼���ݵĴ����W25Q128��λ��; 0x00A00000���ֿ����ʼλ�ã���У׼���ݴ��������ǰ�档

    xXPT2046.lcdWidth  = lcdWidth;
    xXPT2046.lcdHeight = lcdHeight;
    xXPT2046.dir = dir;

#ifdef USE_HAL_DRIVER                                           // HAL�� ����
    // ʹ��GPIO�˿�
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN ;  // ʹ��GPIOA��B��C��D��ʱ��
                                        
    /// ��ʼ�� CS����
    GPIO_InitStruct.Pin   = XPT2046_CS_PIN;                      // ���ű��
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;                 // ���Ź���ģʽ���������
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                         // �ڲ�������������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;           // ���ŷ�ת���ʣ����
    HAL_GPIO_Init(XPT2046_CS_GPIO, &GPIO_InitStruct);            // ��ʼ��

    CS_HIGH;                                                     // ����Ƭѡ����ֹ�����

    GPIO_InitStruct.Pin   = XPT2046_CLK_PIN;                     // CLK
    HAL_GPIO_Init(XPT2046_CLK_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = XPT2046_MOSI_PIN;                    // MOSI
    HAL_GPIO_Init(XPT2046_MOSI_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = XPT2046_MISO_PIN;                    // MISO
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;                     // ��������
    HAL_GPIO_Init(XPT2046_MISO_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = XPT2046_IRQ_PIN;                      // IRQ�ź�����
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                      // �����������ź�ָʾ���ţ���ʹ���ж�
    HAL_GPIO_Init(XPT2046_IRQ_GPIO, &GPIO_InitStruct);
#endif

#ifdef USE_STDPERIPH_DRIVER                                      // ��׼�� ����
    // ʹ��ʱ��
    RCC_AHB1PeriphClockCmd(XPT2046_IRQ_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_CS_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_CLK_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_MOSI_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_MISO_PORT_CLK, ENABLE);
    // ����CS���Ź���ģʽ
    GPIO_InitStruct.GPIO_Pin   = XPT2046_CS_PIN;                 // ѡ��Ҫ���Ƶ�GPIO����
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;                  // ����ģʽ�����ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                  // ������ͣ��������
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;                   // ��������  ����ģʽ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;                // �������ʣ�2MHz
    GPIO_Init(XPT2046_CS_GPIO, &GPIO_InitStruct);                // ���ÿ⺯����ʹ����������ó�ʼ��GPIO
    CS_HIGH;                                                     // ����Ƭѡ����ֹ�����
    // CLK                                                       
    GPIO_InitStruct.GPIO_Pin   = XPT2046_CLK_PIN;                // CLK
    GPIO_Init(XPT2046_CLK_GPIO, &GPIO_InitStruct);               
    // MOSI                                                      
    GPIO_InitStruct.GPIO_Pin   = XPT2046_MOSI_PIN;               // MOSI
    GPIO_Init(XPT2046_MOSI_GPIO, &GPIO_InitStruct);              
    // MISO, ��������                                            
    GPIO_InitStruct.GPIO_Pin   = XPT2046_MISO_PIN;               
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;                   // MISO, ��������
    GPIO_Init(XPT2046_MISO_GPIO, &GPIO_InitStruct);              
    // IRQ                                                       
    GPIO_InitStruct.GPIO_Pin  = XPT2046_IRQ_PIN;                 
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;                    // �����������ź�ָʾ���ţ���ʹ���ж�
    GPIO_Init(XPT2046_IRQ_GPIO, &GPIO_InitStruct);
#endif

    CLK_LOW ;                                                    // XPT2046ʱ��Ҫ��CLK ��ʱ�͵�ƽ�������ز������ݣ��½��ظı�����
    MOSI_0;                                                      // XPT2046ʱ��Ҫ��MOSI��ʱ�͵�ƽ
    CS_LOW;                                                      // ����Ƭѡ��ʹXTP2046��ʼͨ��

    // ͨ���жϷ���ȷ��У׼���ݵĴ洢��ַ
    uint8_t flashDATA[20] = {0};                                 // ���ڴ�Ŷ�ȡ��У׼����
    uint32_t addr = 0;                                           // У׼���ݵĵ�ַ
    if (xXPT2046.dir == 0)                                       // ����
        addr = xXPT2046.dataAddr + 0;                            // ���ݵ�ַ��ƫ�ƣ�0 ���ֽ�
    if (xXPT2046.dir == 1)                                       // ����
        addr = xXPT2046.dataAddr + 20;                           // ���ݵ�ַ��ƫ�ƣ�20���ֽ�

    // ��ȡ�洢��У׼����, ����Ƿ���У׼
    W25Q128_ReadData(addr, flashDATA, 20);                       // ��ȡ����
    if ((flashDATA[0] == 'O') && (flashDATA[1] == 'K'))          // �ж��Ƿ������У׼������
    {
        memcpy(&xXPT2046.xfac, flashDATA + 2, 4);
        xXPT2046.xoff = *(short *)(flashDATA + 6);
        memcpy(&xXPT2046.yfac, flashDATA + 10, 4);
        xXPT2046.yoff = *(short *)(flashDATA + 14);
    }
    else                                                         // û�ж�ȡ���ɵ�У׼����
    {
#if 1
        if (xXPT2046.dir == 0)                                   // ����
            writeCorrectingData(0.0675f, 0.091f, -18, -19);      // Ԥ��д��������У�����ݣ��Լ�������ʱ�ֶ�У������
        else                                                     // ����
            writeCorrectingData(0.09184f, 0.077956f, 338, -45);  // Ԥ��д�������У�����ݣ��Լ�������ʱ�ֶ�У������
        delayUS(10000);                                          // ������ʱ
        SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;                // ��λ����������
#else
        XPT2046_ReCalibration();                                 // �ֶ�����У׼
#endif
    }

    XPT2046_Cmd(ENABLE);                                         // �򿪴������
}



/******************************************************************************
 * �������� XPT2046_ReCalibration
 * ��  �ܣ� ����У׼������,
 *          ����У׼�����ݴ����ڲ�FLASH, �Է����´ε���
 * ��  ���� ��
 * ��  �أ� 0_У׼�ɹ�
 *          1_У׼ʧ��
 ******************************************************************************/
uint8_t  XPT2046_ReCalibration(void)
{
    uint16_t pixelOff = 30;   // ƫ������,������ʮ��
    uint16_t adcX1, adcX2, adcX3, adcX4, adcY1, adcY2, adcY3, adcY4; // ��¼У׼�����е�����ֵ
    float xfac = 0;
    float yfac = 0;
    short xoff = 0;
    short yoff = 0;
    uint16_t crossX = 0;      // ���ڻ�ʮ����
    uint16_t crossY = 0;      // ���ڻ�ʮ����
    char strTemp[30];
    uint16_t lcdWidth  = xXPT2046.lcdWidth;
    uint16_t lcdHeight = xXPT2046.lcdHeight;

    printf("\r\r��������ʼ����У׼....\r");
    printf("��ʹ�ñʼ⣬���ε����ɫ���!\r\r");
    LCD_Fill(0, 0, lcdWidth, lcdHeight, BLACK);
    LCD_String(20, 90,  "Please use the pen , ", 16, WHITE, BLACK);
    LCD_String(20, 115, "and click on the reticle !", 16, WHITE, BLACK);

    // ���Ͻ�
    crossX = pixelOff;
    crossY = pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // �򿪴������(����)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // ��ʮ��
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // �ȴ�����
    XPT2046_Cmd(DISABLE);                                             // �رմ������(����)
    adcX1 = xXPT2046 .adcX;                                           // ��¼��ȡ�õ�adcֵ
    adcY1 = xXPT2046.adcY;                                            // ��¼��ȡ�õ�adcֵ
    LCD_Cross(crossX, crossY, 20, BLACK);                             // Ĩȥʮ��
    sprintf(strTemp, "X:%d", adcX1);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // ��ʾ
    sprintf(strTemp, "Y:%d", adcY1);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // ��ʾ
    delayUS(800000);

    // ���Ͻ�
    crossX = lcdWidth - pixelOff;
    crossY = pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // �򿪴������(����)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // ��ʮ��
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // �ȴ�����
    XPT2046_Cmd(DISABLE);                                             // �رմ������(����)
    adcX2 = xXPT2046 .adcX;                                           // ��¼��ȡ�õ�adcֵ
    adcY2 = xXPT2046.adcY;                                            // ��¼��ȡ�õ�adcֵ
    LCD_Cross(crossX, crossY, 20, BLACK);                             // Ĩȥʮ��
    sprintf(strTemp, "X:%d", adcX2);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // ��ʾ
    sprintf(strTemp, "Y:%d", adcY2);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // ��ʾ
    delayUS(800000);

    // ���½�
    crossX = pixelOff;
    crossY = lcdHeight - pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // �򿪴������(����)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // ��ʮ��
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // �ȴ�����
    XPT2046_Cmd(DISABLE);                                             // �رմ������(����)
    adcX3 = xXPT2046 .adcX;                                           // ��¼��ȡ�õ�adcֵ
    adcY3 = xXPT2046.adcY;                                            // ��¼��ȡ�õ�adcֵ
    LCD_Cross(crossX, crossY, 20, BLACK);                             // Ĩȥʮ��
    sprintf(strTemp, "X:%d", adcX3);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // ��ʾ
    sprintf(strTemp, "Y:%d", adcY3);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // ��ʾ
    delayUS(800000);

    // ���½�
    crossX = lcdWidth - pixelOff;
    crossY = lcdHeight - pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // �򿪴������(����)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // ��ʮ��
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // �ȴ�����
    XPT2046_Cmd(DISABLE);                                             // �رմ������(����)
    adcX4 = xXPT2046 .adcX;                                           // ��¼��ȡ�õ�adcֵ
    adcY4 = xXPT2046.adcY;                                            // ��¼��ȡ�õ�adcֵ
    LCD_Cross(crossX, crossY, 20, BLACK);                             // Ĩȥʮ��
    sprintf(strTemp, "X:%d", adcX4);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // ��ʾ
    sprintf(strTemp, "Y:%d", adcY4);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // ��ʾ
    delayUS(400000);

    //timeCNT=0;
    // ȡadcX��adcY��ƽ��ֵ; �����ȡƽ��ֵ, �ڶԽǻ�����ʮ���߼���
    adcX1 = (adcX1 + adcX3) / 2;
    adcX2 = (adcX2 + adcX4) / 2;

    adcY1 = (adcY1 + adcY2) / 2;
    adcY2 = (adcY3 + adcY4) / 2;

    xfac = (float)(pixelOff - (lcdWidth - pixelOff)) / (adcX1 - adcX2);   // ��������LCD���������ϵ��,  xfac=(float)(20-320)/(t1x-t2x);
    yfac = (float)(pixelOff - (lcdHeight - pixelOff)) / (adcY1 - adcY2);
    xoff = (lcdWidth - xfac * (adcX1 + adcX2)) / 2;                       // ���ص�ƫ��ֵ, xoff=(320-xfac*(t1x+t2x))/2;
    yoff = (lcdHeight - yfac * (adcY1 + adcY2)) / 2;

    // ����FLASH��
    writeCorrectingData(xfac, yfac, xoff, yoff);

    xXPT2046.xfac = xfac;
    xXPT2046.xoff = xoff;
    xXPT2046.yfac = yfac;
    xXPT2046.yoff = yoff;

    printf(">>>У׼���! ����ϵ����ƫ��ֵ�Ѵ����ڲ�FLASH, ��ַ:0x%X\r\r", xXPT2046.dataAddr);

    SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;  // ��λ����������
    return 0;
}



/******************************************************************************
 * �������� XPT2046_Cmd
 * ��  �ܣ� ������⿪��
 *          ���ڼ��ȽϺ�ʱ, �ڲ�ʹ�ô�����״̬��, ���Թرռ���Խ�ʡоƬ��Դ
 *          �˿���״̬, ֻ������XPT2046_TouchHandler();
 * ��  ���� 0_�رմ������ļ��,�Խ�ʡ��Դ
 *          1_�򿪴������
 * ��  �أ�
 ******************************************************************************/
void XPT2046_Cmd(uint8_t status)
{
    if (status != 0)
    {
        xXPT2046 .EN = 1;
    }
    else
    {
        xXPT2046.EN = 0;
    }
}



/******************************************************************************
 * �������� XPT2046_IsPressed
 * ��  �ܣ� �жϴ������Ƿ��а���
 * ��  ���� ��
 * ��  �أ� 0-δ���¡�1-������
 ******************************************************************************/
uint8_t XPT2046_IsPressed(void)
{
    static uint8_t status = 0;

    if (xXPT2046.EN == 0)             // ������⿪��; ��ʼ����Ĭ���ǿ�����; �����ֶ��رգ���ʵ��һЩ������Ȼ�����ֶ�����
        return 0;

    status = IRQ_READ ? 0 : 1 ;       // ������ʱΪ�ߵ�ƽ������ʱΪ�͵�ƽ
    if (status == 1)                  // ����Ѱ���
    {
        readAdcXY();                  // ��ȡXPT2046�İ���λ��(��ѹֵ)
        adcXYToLcdXY();               // �������ʾ��������ֵ; ������XYֵ����ͨ����XPT2046_GetX()��XPT2046_GetY()��ȡ;
        uint16_t x1 = xXPT2046.lcdX;
        uint16_t y1 = xXPT2046.lcdY;

        readAdcXY();                  // ��ȡXPT2046�İ���λ��(��ѹֵ)
        adcXYToLcdXY();               // �������ʾ��������ֵ; ������XYֵ����ͨ����XPT2046_GetX()��XPT2046_GetY()��ȡ;
        uint16_t x2 = xXPT2046.lcdX;
        uint16_t y2 = xXPT2046.lcdY;

        // �����ֵ��
        uint8_t x, y;
        if (x1 > x2)
            x = x1 - x2;
        else
            x = x2 - x1;

        if (y1 > y2)
            y = y1 - y2;
        else
            y = y2 - y1;

        if (x > 3 || y > 3)
            return 0;

        // ����ƽ��ֵ
        xXPT2046.lcdX = (x1 + x2) >> 1;
        xXPT2046.lcdY = (y1 + y2) >> 1;
    }    
    
    return  status ;      // ���أ�0-δ���¡�1-������             
}



/******************************************************************************
 * �������� XPT2046_GetX
 * ��  �ܣ� ��ȡ����λ�õ�����ֵ (X)
 * ��  ���� ��
 * ��  �أ� ����ֵ (X)
 ******************************************************************************/
uint16_t  XPT2046_GetX(void)
{
    return xXPT2046.lcdX;
}



/******************************************************************************
 * �������� XPT2046_GetY
 * ��  �ܣ� ��ȡ����λ�õ�����ֵ (Y)
 * ��  ���� ��
 * ��  �أ� ����ֵ (Y)
 ******************************************************************************/
uint16_t  XPT2046_GetY(void)
{
    return xXPT2046.lcdY;
}



/******************************************************************************
 * �������� XPT2046_TouchDown
 * ��  �ܣ� ��������ʱ�Ĵ���
 *          �հ׺���, �û����б�д����
 * ��  ����
 * ��  �أ�
 ******************************************************************************/
void XPT2046_TouchDown(void)
{
//    // ʾ���ڴ����㻭һ����ɫ��
//    LCD_DrawPoint(xXPT2046.lcdX, xXPT2046.lcdY, YELLOW);
//    // ʾ������������ʾ��LCD
//    static char strTem[20];
//    sprintf(strTem, "%3d  %3d", xXPT2046.lcdX, xXPT2046.lcdY);
//    LCD_String(180, 290, strTem, 12, WHITE, BLACK);
}



/******************************************************************************
 * �������� XPT2046_TouchUp
 * ��  �ܣ� ��������ʱ�Ĵ���
 *          �հ׺���, �û����б�д����
 * ��  ����
 * ��  �أ�
 ******************************************************************************/
void XPT2046_TouchUp(void)
{


}

