/***********************************************************************************************************************************
 ** ���ļ����ơ�  bsp_w25qxx.c
 ** ����д��Ա��  ħŮ�������Ŷ�
 ** �����·���   
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** ���ļ����ܡ�  ��ʼ��GPIO��SPI, �����ܺ���
 ** ������ƽ̨��  STM32F407 + ��׼��v1.8 + keil5
 **
 ** �����¼�¼��  2023-01-27  ����ע�͡������ʽ 
 **               2022-12-29  �����ļ�����F103��ֲ��F407
************************************************************************************************************************************/
#include "bsp_W25Q128.h"



//W25Qϵ��оƬ�ͺŷ���ֵ
#define    W25Q80            0XEF13
#define    W25Q16            0XEF14
#define    W25Q32            0XEF15
#define    W25Q64            0XEF16
#define    W25Q128           0XEF17
#define    W25Q256           0XEF18
//#define  W25Qxx    65519   // �ܶ�ʱ���������غ�����Ķ���65519
#define    W25Q128_CS_HIGH    (W25Q128_CS_GPIO -> BSRR =  W25Q128_CS_PIN)
#define    W25Q128_CS_LOW     (W25Q128_CS_GPIO -> BSRR =  W25Q128_CS_PIN << 16)

xW25Q_TypeDef  xW25Q128;      // ����ȫ�ֽṹ��, ���ڼ�¼w25qxx��Ϣ





// 5_1 ����1�ֽ�,����1�ֽ�
// SPIͨ��,ֻһ������:��DRд���������ֵ,ͬ����������!д�����,������ʱ��ͼ��. ��Ϊ����,��Ϊ�շ�ͬ��,�����շ����ж�Ҳ���ÿ�,δ��֤�����ж϶��乤����Ӱ��.
static uint8_t  sendByte(uint8_t d)
{
    uint16_t retry = 0;

    while ((W25Q128_SPI ->SR & 2) == 0)      // �ȴ�������Ϊ��
    {
        retry++;
        if (retry > 1000)    return 0;
    }
    W25Q128_SPI ->DR = d;

    retry = 0;
    while ((W25Q128_SPI->SR & 1) == 0)       // �ȴ�����������
    {
        retry++;
        if (retry > 1000)    return 0;
    }
    return W25Q128_SPI->DR ;
}



// 5_2 дʹ��
static void writeEnable()
{
    W25Q128_CS_LOW ;
    sendByte(0x6);                           // ����: Write Enable : 06h
    W25Q128_CS_HIGH ;
}



// 5_3 �ȴ�����
static void WaitReady()
{
    W25Q128_CS_LOW ;

    sendByte(0x05);                          // ����: Read Status Register : 05h
    while (sendByte(0xFF) & 1) {}            // ֻҪ���Ͷ�״̬�Ĵ���ָ�оƬ�ͻ�����������������µ�״̬�Ĵ������� ��ֱ���յ�ͨ�ŵ�ֹͣ�źš�

    W25Q128_CS_HIGH ;
}



// 5_4 ����һ������, ÿ����>150ms
static void eraseSector(uint32_t addr)
{
    if (xW25Q128.FlagInit == 0) return;      // ���W25Qxx��ʼ��ʧ�ܣ���������⣬��ֹ����

    addr = addr * 4096;                      // �ӵڼ�������ʼ

    writeEnable();
    WaitReady();
    // ����
    W25Q128_CS_LOW ;
    sendByte(0x20);                          // ����: Sector Erase(4K) : 20h
    sendByte((uint8_t)(addr >> 16));
    sendByte((uint8_t)(addr >> 8));
    sendByte((uint8_t)addr);
    W25Q128_CS_HIGH ;

    WaitReady();
}



// 5_5 д����. Ҫ��ҳд��
static void writeSector(uint32_t addr, uint8_t *p, uint16_t num)
{
    if (xW25Q128.FlagInit == 0) return;           // ���W25Qxx��ʼ��ʧ�ܣ���������⣬��ֹ����

    uint16_t pageRemain = 256;                    // ��Ҫ����Ҫ����Ҫ��W25Qxxÿ��ҳ�������д���ֽ���:256�ֽ�;

    for (char i = 0; i < 16; i++)                 // ����:4096bytes, ����ҳ:256bytes, д����Ҫ��16��ҳ����д��
    {
        writeEnable();                            // дʹ��
        WaitReady();                              // �ȴ�����

        W25Q128_CS_LOW ;                          // �͵�ƽ,��ʼ
        sendByte(0x02);                           // ����: page program : 02h , ÿ��дҳ������󻺴�256�ֽ�
        sendByte((uint8_t)(addr >> 16));          // ��ַ
        sendByte((uint8_t)(addr >> 8));
        sendByte((uint8_t)addr);
        for (uint16_t i = 0; i < pageRemain; i++) // ����д�������
            sendByte(p[i]);                       // �ߵ�ƽ, ����
        W25Q128_CS_HIGH ;

        WaitReady();                              // �ȴ�����

        p = p + pageRemain;                       // ����ָ������һҳ�ֽ���
        addr = addr + pageRemain ;                // д��ַ����һҳ�ֽ���
    }
}



// ��ȡоƬ�ͺ�
static uint32_t readID(void)
{
    uint16_t Temp = 0;
    W25Q128_CS_LOW;
    sendByte(0x90);//���Ͷ�ȡID����
    sendByte(0x00);
    sendByte(0x00);
    sendByte(0x00);
    Temp |= sendByte(0xFF) << 8;
    Temp |= sendByte(0xFF);
    W25Q128_CS_HIGH;

    xW25Q128.FlagInit  = 1;
    switch (Temp)
    {
        case W25Q16:
            sprintf((char *)xW25Q128.type, "%s", "W25Q16");
            break;
        case W25Q32:
            sprintf((char *)xW25Q128.type, "%s", "W25Q32");
            break;
        case W25Q64:
            sprintf((char *)xW25Q128.type, "%s", "W25Q64");
            break;
        case W25Q128:
            sprintf((char *)xW25Q128.type, "%s", "W25Q128");
            break;
        case W25Q256:
            sprintf((char *)xW25Q128.type, "%s", "W25Q256");          // ע��:W25Q256�ĵ�ַ��4�ֽ�
            break;
        default:
            sprintf((char *)xW25Q128.type, "%s", "Flash�豸ʧ�� !!!");
            xW25Q128.FlagInit = 0;
            printf("��ȡ���Ĵ����ͺ����ݣ�%d\r\n", Temp);
            break;
    }

    if (xW25Q128.FlagInit  == 1)  
        printf("Flash�洢 ���...        �ͺ�:%s\r", xW25Q128.type);  
    else
        printf("���ݴ洢��⣺           �ͺŶ�ȡ�����豸������!\r");
        
    return Temp;
}



// ����ֿ���������ȷ��
static void checkFlagGBKStorage(void)
{
    if (xW25Q128 .FlagInit == 0) return;                  // ���W25Qxx��ʼ��ʧ�ܣ���������⣬��ֹ����

    printf("GBK�ֿ� ����...          ");
    uint8_t sub = 0;
    uint8_t f = 0 ;

    for (uint32_t i = 0; i < 6128640; i = i + 1000000)
    {
        W25Q128_ReadData(GBK_STORAGE_ADDR + i, &f, 1);
        sub = sub + f;                                    // 80 , 0, 98, 79, 0, 1, 0
    }
    xW25Q128.FlagGBKStorage = (sub == 146 ? 1 : 0);       // �ж��Ƿ����ֿ�,�򿪵�ַд����, ��ֹ�ֿⱻ����д���ĸ�

    if (xW25Q128.FlagGBKStorage == 1)
        printf("�ֿ����\r");                             // ����ֿ����
    else
        printf(" �����ֿⲻ����!\r");
}



/******************************************************************************
 * ��  ���� W25qx_Init
 * ��  �ܣ� ��ʼ��W25Q128�������š�SPI
 * ��  ����
 * ����ֵ�� ��ʼ�������0:ʧ�ܡ�1:�ɹ�
 ******************************************************************************/
uint8_t W25Q128_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // CS����ʱ��
    if(W25Q128_CS_GPIO == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if(W25Q128_CS_GPIO == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if(W25Q128_CS_GPIO == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if(W25Q128_CS_GPIO == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if(W25Q128_CS_GPIO == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    if(W25Q128_CS_GPIO == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    if(W25Q128_CS_GPIO == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    // SPI����ʱ��
    if(W25Q128_SCK_GPIO == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if(W25Q128_SCK_GPIO == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if(W25Q128_SCK_GPIO == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if(W25Q128_SCK_GPIO == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if(W25Q128_SCK_GPIO == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    if(W25Q128_SCK_GPIO == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    if(W25Q128_SCK_GPIO == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    // SPIʱ�� 
    if(W25Q128_SPI== SPI1)  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    if(W25Q128_SPI== SPI2)  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; 
    if(W25Q128_SPI== SPI3)  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN; 
    
    // ��������: CS
    GPIO_InitStructure.GPIO_Pin = W25Q128_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(W25Q128_CS_GPIO, &GPIO_InitStructure);

    W25Q128_CS_HIGH;                                     // CS�������ߣ�ֹͣ�ź� 
    
    // ���� SPI����: SCK��MISO��MOSI
    GPIO_InitStructure.GPIO_Pin = W25Q128_SCK_PIN | W25Q128_MISO_PIN | W25Q128_MOSI_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(W25Q128_SCK_GPIO, &GPIO_InitStructure);

    //�������Ÿ���
    GPIO_PinAFConfig(W25Q128_SCK_GPIO,  W25Q128_SCK_PINSOURCE,  W25Q128_SCK_AF);
    GPIO_PinAFConfig(W25Q128_MISO_GPIO, W25Q128_MISO_PINSOURCE, W25Q128_MISO_AF);
    GPIO_PinAFConfig(W25Q128_MOSI_GPIO, W25Q128_MOSI_PINSOURCE, W25Q128_MOSI_AF);

    // ����SPI����ģʽ
    // FLASHоƬ ֧��SPIģʽ0��ģʽ3���ݴ�����CPOL CPHA
    #if 0
    SPI_InitTypeDef  SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(W25Q128_SPI, &SPI_InitStructure);
    SPI_Cmd(W25Q128_SPI, ENABLE);   // ʹ��SPI
    #else
    W25Q128_SPI -> CR1  = 0x1 << 0;       // CPHA:ʱ����λ,0x1=�ڵ�2��ʱ�ӱ��ؽ������ݲ���
    W25Q128_SPI -> CR1 |= 0x1 << 1;       // CPOL:ʱ�Ӽ���,0x1=����״̬ʱ��SCK���ָߵ�ƽ
    W25Q128_SPI -> CR1 |= 0x1 << 2;       // ����ģʽ:         1 = ������
    W25Q128_SPI -> CR1 |= 0x0 << 3;       // �����ʿ���[5:3]:  0 = fPCLK /2
    W25Q128_SPI -> CR1 |= 0x0 << 7;       // ֡��ʽ:           0 = �ȷ���MSB
    W25Q128_SPI -> CR1 |= 0x1 << 9;       // ������������� :  1 = ʹ���������������(���NSS)
    W25Q128_SPI -> CR1 |= 0x1 << 8;       // �ڲ�������ѡ��,����9λ����(ʧ���ڲ�NSS)
    W25Q128_SPI -> CR1 |= 0x0 << 11;      // ����֡��ʽ,       0 = 8λ
    W25Q128_SPI -> CR1 |= 0x1 << 6;       // SPIʹ��           1 = ʹ������
    #endif

    readID();                             // ��ȡоƬ�ͺ�,�ж�ͨѶ�Ƿ�����
    checkFlagGBKStorage();                // ����ֿ�
    
    if(xW25Q128.FlagInit)
        return 1;                         // ��ʼ���ɹ�������:1
    else
        return 0;                         // ��ʼ��ʧ�ܣ�����:0
}



/******************************************************************************
 * ��  ���� W25Q128_ReadData
 * ��  �ܣ� ��ȡ����
 * ��  ���� uint32_t addr��������W25Q128�ڵĵ�ַ
 *          uint8_t *p   �����ݻ����ַ
 *          uint16_t num ��������ȡ���ֽ���
 * ����ֵ�� ��
 ******************************************************************************/
void W25Q128_ReadData(uint32_t addr, uint8_t *p, uint16_t num)
{
    if (xW25Q128 .FlagInit == 0) return;  // ���W25Qxx��ʼ��ʧ�ܣ���������⣬��ֹ����

    W25Q128_CS_LOW ;
    sendByte(0x03);                       // ���Ͷ�ȡ���� 03h
    sendByte((uint8_t)(addr >> 16));
    sendByte((uint8_t)(addr >> 8));
    sendByte((uint8_t)addr);

    for (uint32_t i = 0; i < num; i++)
    {
        p[i] = sendByte(0xFF);
    }

    W25Q128_CS_HIGH ;
}



/******************************************************************************
 * �������� W25Q128_Write
 * ��  �ܣ� ��addr���𣬶�ȡnum���ֽڣ���ŵ�����p
 * ��  ���� uint32_t  addr   д���ַ         (W25Q128 ֻ��3�ֽ�, W25Q256��4�ֽ�)
 *          uint8_t  *pData  Ҫд������ݴ洢��
 *          uint16_t  num    д����ֽ���
 * ��  �أ� ��
 * ��  ע�� ������_2020��12��15��
 ******************************************************************************/
uint8_t W25Q128_buffer[4096];                         // ����һ���ڴ�ռ�

void W25Q128_WriteData(uint32_t addr, uint8_t *pData, uint16_t num)
{
    if (xW25Q128.FlagInit == 0) return ;              // ���w25qxx�豸��ʼ��ʧ�ܣ�����������������ֹ����

    // �ֿ��д����, ��ֹ�ֿⱻ����д���ĸ�
    if (((addr + num) > 0x00A00000) && (xW25Q128.FlagGBKStorage == 1))
    {
        printf("Ҫд����������ֿ����ݴ洢���ڣ����������β���!!\r");
        return;
    }

    uint32_t  secPos      = addr / 4096;              // ������ַ,�ڼ�������
    uint16_t  secOff      = addr % 4096;              // ��ʼ��ʼƫ���ֽ���: �����������ĵڼ��ֽڴ��
    uint16_t  secRemain   = 4096 - secOff;            // ����ʣ��ռ��ֽ��� ,�����жϹ�����������µ�����
    uint8_t  *buf = W25Q128_buffer;                   // ԭ�Ӹ����,Ϊʲô��ֱ��ʹ��������������. 

    //spiInit();                                      // ÿ�ζ�дǰ������������SPI���������豸����һSPIʱ�����ò�ͬ
    if (num <= secRemain) secRemain = num;
    while (1)
    {
        W25Q128_ReadData(secPos * 4096, buf, 4096);   // ��ȡ�������ݵ�����

        eraseSector(secPos);                          // ������
        for (uint16_t i = 0; i < secRemain ; i++)     // ԭʼ����д�뻺��
            buf[secOff + i] = pData[i];
        writeSector(secPos * 4096, buf, 4096);        // ��������д���豸

        if (secRemain == num)                         // ��ȫ��д��
            break;
        else
        {
            // δд��
            pData = pData + secRemain ;               // ԭʼ����ָ��ƫ��
            secPos ++;                                // ������
            secOff = 0;                               // ��ƫ��λ,������������ʼ��ַ
            num = num - secRemain ;                   // ʣ��δд�ֽ���
            secRemain = (num > 4096) ? 4096 : num;    // ����������д���ֽ���
        }
    }
}




/******************************************************************************
 * �������� W25Q128_ReadFontData
 * ��  �ܣ� ��w25Q128���ֿ��ж�ȡ��������ģ����
 * ��  ���� uint8_t *pFont     ����
 *          uint8_t  size      �����С 12/16/24/32
 *          uint8_t *fontData  ��ȡ������ģ��������
 * ��  �أ� ��
 * ��  ע�� ������_2024��02��05��
 ******************************************************************************/
void W25Q128_ReadFontData(uint8_t *pFont, uint8_t size, uint8_t *fontData)
{
    uint8_t qh, ql;
    uint32_t foffset;
    uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size);                           // ���㺺�ֵ����С����λ�ֽ���
                                                                                         
    qh = *pFont;                                                                          // ����GBK�ĵ�һ���ֽ�
    ql = *(++pFont);                                                                      // ����GBK�ĵڶ����ֽ�
                                                                                         
    if (qh < 0x81 || ql < 0x40 || ql == 0xff || qh == 0xff)                               // �����ֿ��ڵĺ��֣����������ʾ����λ��
    {                                                                                    
        for (uint8_t i = 0; i < csize; i++) *fontData++ = 0x00;                           // �������
        return;                                                                           // ����
    }                                                                                    
                                                                                         
    if (ql < 0x7f)                                                                        // ����Ҫ��ȡ�ĺ������ֿ��е�ƫ��λ��
         ql -= 0x40;                                                                     
    else                                                                                 
        ql -= 0x41;                                                                      
    qh -= 0x81;                                                                          
    foffset = ((unsigned long)190 * qh + ql) * csize;                                     // �õ��������ֿ��е�ƫ��λ��
                                                                                         
    switch (size)                                                                         // ������Ĳ�ͬ���ڲ�ͬ�ֿ��ȡ�������
    {
        case 12:
            W25Q128_ReadData(foffset + GBK_STORAGE_ADDR,            fontData, csize);     // 12������
            break;
        case 16:
            W25Q128_ReadData(foffset + GBK_STORAGE_ADDR + 0x0008c460, fontData, csize);   // 16������
            break;
        case 24:
            W25Q128_ReadData(foffset + GBK_STORAGE_ADDR + 0x001474E0, fontData, csize);   // 24������
            break;
        case 32:
            W25Q128_ReadData(foffset + GBK_STORAGE_ADDR + 0x002EC200, fontData, csize);   // 32������
            break;
    }
}

