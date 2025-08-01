/***********************************************************************************************************************************
**���Ա����ӡ�  ħŮ������    https://demoboard.taobao.com
***********************************************************************************************************************************
**���ļ����ơ�  bsp_IICSoft.c
**
**������������  ģ��IICʱ��
**              ���幦�ܺ���
**
**�����¼�¼��  2023-11-10  �Ż����ֽڶ�д����
**              2020-03-05  ����
**              2021-05-03  �����ļ���ʽ��ע�͸�ʽ
**
***********************************************************************************************************************************/
#include "bsp_I2CSoft.h"






/************************************************
 ** �궨��_���ŵ�ƽ���Ƽ򻯡����ӿɴ���ɶ���
 ***********************************************/
#define  I2C_SCL_1       ( I2C_SCL_GPIO->BSRR = I2C_SCL_PIN )        // SCL �ø�
#define  I2C_SCL_0       ( I2C_SCL_GPIO->BSRR = I2C_SCL_PIN << 16 )  // SCL �õ�
#define  I2C_SDA_1       ( I2C_SDA_GPIO->BSRR = I2C_SDA_PIN )        // SDA �ø�
#define  I2C_SDA_0       ( I2C_SDA_GPIO->BSRR = I2C_SDA_PIN << 16 )  // SDA �õ�
                                                            

#define  I2C_SDA_READ()  ( I2C_SDA_GPIO->IDR & I2C_SDA_PIN )         // ��SDA��ƽ״̬






/*****************************************************************************
 * ȫ�ֺ궨��
 * �����޸�
****************************************************************************/
#define  I2C_WR              0                 // д����λ
#define  I2C_RD              1                 // ������λ





/*****************************************************************************
 ** �ڲ����� ����
*****************************************************************************/
static void delay(void);





/********************************************************************************************************
 * ��  ����delay
 * ��  �ܣ�I2C����λ�ӳ٣����400KHz
 * ��  ������
 * ����ֵ����
 * ��  ע��ϵͳʱ��168MHz, �������20~250��������ͨ��
********************************************************************************************************/
static void delay(void)
{
    uint8_t i;
    for (i = 0; i < 30; i++);
}



/********************************************************************************************************
 * ��  ����I2CSoft_Start
 * ��  �ܣ������߲�����ʼ�ź�(SCL�ߵ�ƽ�ڼ䣬SDA�ɸ��������)
 * ��  ������
 * ����ֵ����
*********************************************************************************************************/
void I2CSoft_Start(void)
{
    I2C_SDA_1;
    delay();  //
    I2C_SCL_1;
    delay();
    I2C_SDA_0;
    delay();
    I2C_SCL_0;
    delay();
}



/********************************************************************************************************
 * ��  ����I2CSoft_Start
 * ��  �ܣ������߲���ֹͣ�ź�(��SCL�ߵ�ƽ��䣬SDA�ɵ��������)
 * ��  ������
 * ����ֵ����
********************************************************************************************************/
void I2CSoft_Stop(void)
{
    I2C_SDA_0;
    delay();   //
    I2C_SCL_1;
    delay();
    I2C_SDA_1;
    delay();   //
}



/********************************************************************************************************
 * ��  ����I2CSoft_WaitAck
 * ��  �ܣ������߲���һ��ʱ�ӣ���ȡ�ӻ���ACKӦ���ź�
 * ��  ������
 * ����ֵ��0-Ӧ��1-��Ӧ��
 ********************************************************************************************************/
uint8_t I2CSoft_WaitAck(void)
{
    uint8_t ack;

    I2C_SDA_1;           // �ͷ�SDA����
    delay();
    I2C_SCL_1;           // SC = 1, �ȴ������᷵��ACKӦ��
    delay();
    if (I2C_SDA_READ())  // ��ȡSDA��ƽ
    {
        ack = 1;         // �ߵ�ƽ������Ӧ��
    }
    else
    {
        ack = 0;         // �͵�ƽ����Ӧ��
    }
    I2C_SCL_0;           // ����Ӧ��ʱ��
    delay();
    return ack;          // ���أ�0-Ӧ��1-��Ӧ��
}



/********************************************************************************************************
 * ��  ����I2CSoft_Ack
 * ��  �ܣ������߲���һ��ACK�ź�
 * ��  ������
 * ����ֵ����
********************************************************************************************************/
void I2CSoft_Ack(void)
{
    I2C_SDA_0;
    delay();
    I2C_SCL_1;
    delay();
    I2C_SCL_0;
    delay();
    I2C_SDA_1;
}



/*******************************************************************************************************
 * ��  ����I2CSoft_NAck
 * ��  �ܣ������߲���1����Ӧ��NACK�ź�
 * ��  ������
 * ����ֵ����
*******************************************************************************************************/
void I2CSoft_NAck(void)
{
    I2C_SDA_1;
    delay();
    I2C_SCL_1;
    delay();
    I2C_SCL_0;
    delay();
}



/********************************************************************************************************
 * ��  ����I2CSoft_SendByte
 * ��  �ܣ������߷���8bit����
 * ��  ����uint8_t _uData�� ���͵��ֽ�
 * ����ֵ����
********************************************************************************************************/
void I2CSoft_SendByte(uint8_t _uData)
{
    for (uint8_t i = 0; i < 8; i++)  // �ɸ�λ����λ����λ������ƽ
    {
        if (_uData & 0x80)
            I2C_SDA_1;
        else
            I2C_SDA_0;
        _uData <<= 1;                // ����1λ
        delay();
        I2C_SCL_1;
        delay();
        I2C_SCL_0;
        delay();
        if (i == 7)                  // ���һλ
        {
            I2C_SDA_1;               // �ͷ�����
        }
    }
}



/********************************************************************************************************
 * ��  ����I2CSoft_ReadByte
 * ��  �ܣ������߶�ȡ8bit����
 * ��  ������
 * ����ֵ������������
*********************************************************************************************************/
uint8_t I2CSoft_ReadByte(void)
{
    uint8_t data = 0;

    for (uint8_t i = 0; i < 8; i++)  // �ɸ�λ����
    {
        data <<= 1;
        I2C_SCL_1;
        delay();
        if (I2C_SDA_READ())
        {
            data++;
        }
        I2C_SCL_0;
        delay();
    }
    return data;
}



/*********************************************************************************************************
 * ��  ����I2CSoft_CheckDevice
 * ��  �ܣ����I2C�ӻ��豸�Ƿ�����Ӧ
 * ��  ����uint8_t _uAddr  �豸��I2C���ߵ�ַ
 * ����ֵ��0-��⵽�豸��Ӧ��1-δ��⵽�豸��Ӧ
********************************************************************************************************/
uint8_t I2CSoft_CheckDevice(uint8_t _uAddr)
{
    uint8_t ucAck;

    I2CSoft_Stop();                    // ������һ��ֹͣ�ź�
    I2CSoft_Start();                   // �����ź�

    I2CSoft_SendByte(_uAddr | I2C_WR); // ����7λ�ӻ���ַ+��д����λ(0-д��1-��)
    delay();
    ucAck = I2CSoft_WaitAck();         // ����豸Ӧ���źţ�0-Ӧ��1-��Ӧ��

    I2CSoft_Stop();                    // ����ֹͣ�ź�
    return ucAck;                      // 0-��⵽�豸��Ӧ��1-δ��⵽�豸��Ӧ
}



/*****************************************************************************
 * ��  ����I2CSoft_Init
 * ��  �ܣ���ʼ��ģ��IIC����
 * ��  ������
 * ����ֵ����
*****************************************************************************/
void I2CSoft_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // ʹ��SCL���Ŷ˿�ʱ��
    if (I2C_SCL_GPIO == GPIOA)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    if (I2C_SCL_GPIO == GPIOB)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    if (I2C_SCL_GPIO == GPIOC)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    if (I2C_SCL_GPIO == GPIOD)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    if (I2C_SCL_GPIO == GPIOE)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    if (I2C_SCL_GPIO == GPIOF)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    if (I2C_SCL_GPIO == GPIOG)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    // ʹ��SDA���Ŷ˿�ʱ��
    if (I2C_SDA_GPIO == GPIOA)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    if (I2C_SDA_GPIO == GPIOB)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    if (I2C_SDA_GPIO == GPIOC)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    if (I2C_SDA_GPIO == GPIOD)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    if (I2C_SDA_GPIO == GPIOE)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    if (I2C_SDA_GPIO == GPIOF)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    if (I2C_SDA_GPIO == GPIOG)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = I2C_SCL_PIN ;      // SCL���ű��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // ���ŷ�ת����
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     // ����ģʽ�����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // ���ģʽ����©
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;      // ����������
    GPIO_Init(I2C_SCL_GPIO, &GPIO_InitStructure);      // ��ʼ��

    GPIO_InitStructure.GPIO_Pin   =  I2C_SDA_PIN;      // SDA���ű��
    GPIO_Init(I2C_SCL_GPIO, &GPIO_InitStructure);      // ��ʼ��

    I2CSoft_Stop();                                    // ������һ��ֹͣ�ź�
    // I2CSoft_Start();                                // �����ź�
}



/*********************************************************************************************************
 * ��  ����I2CSoft_ReadBytes
 * ��  �ܣ���ȡָ������������
 * ��  ����uint8_t     _uSlaveAddr  �豸��ַ
 *         uint8_t     _uDataAddr   ���ݵ�ַ
 *         uint8_t    *_puDataBuf   ������ݵĻ����ַ
 *         uint16_t    _usNum       Ҫ��ȡ���ֽ���
 * ����ֵ��0-ʧ�ܣ�1-�ɹ�
********************************************************************************************************/
uint8_t I2CSoft_ReadBytes(uint8_t _uSlaveAddr, uint8_t _uDataAddr, uint8_t *_puDataBuf, uint16_t _usNum)
{
    I2CSoft_Start();                                // 1-��ʼ�ź�

    I2CSoft_SendByte(_uSlaveAddr | I2C_WR);         // 2-���ʹӻ���ַ+д���ƣ�ע�⣺������д���� 0-д��1-��
    if (I2CSoft_WaitAck())                          // �ȴ�ACK��ע�⣺ÿ����һ���ֽڣ���Ҫȷ�ϴӻ�Ӧ��
        goto ReadBytes_ACK_Fail;                    // �ӻ���Ӧ��, ����ʧ�ܴ���(����ֹͣ�źš�����0)

    I2CSoft_SendByte((uint8_t)_uDataAddr);          // 3-�������ݵ�ַ��ע�⣺24C02ֻ��256�ֽڣ�ֻ�跢��1�����ݵ�ֵַ�����24C04�����Ͼ�Ҫ�����������ݵ�ֵַ
    if (I2CSoft_WaitAck())                          // �ȴ�ACK��ע�⣺ÿ����һ���ֽڣ���Ҫȷ�ϴӻ�Ӧ��
        goto ReadBytes_ACK_Fail;                    // �ӻ���Ӧ��, ����ʧ�ܴ���(����ֹͣ�źš�����0)

    I2CSoft_Start();                                // 4-�ٴβ�����ʼ�źţ���ʼ�����ݴ���

    I2CSoft_SendByte(_uSlaveAddr | I2C_RD);         // 5-�ٴη��ʹӻ���ַ+д���ƣ�ע�⣺�����Ƕ�����
    if (I2CSoft_WaitAck())                          // �ȴ�ACK��ע�⣺ÿ����һ���ֽڣ���Ҫȷ�ϴӻ�Ӧ��
        goto ReadBytes_ACK_Fail;                    // �ӻ���Ӧ��, ����ʧ�ܴ���(����ֹͣ�źš�����0)

    for (uint16_t i = 0; i < _usNum; i++)           // 6-��ʼ��ȡ��Ҫ���ֽ���, ע�⣺����Ҫ��д���׵�ַ�Ϳ��������ض�ȡ
    {
        _puDataBuf[i] = I2CSoft_ReadByte();         // ���ֽ�
        if (i != _usNum - 1)                        // ÿ��1���ֽڣ���Ҫ����Ack�����1���ֽڷ���Nack
            I2CSoft_Ack();                          // �����߲���ACK�ź�
        else
            I2CSoft_NAck();                         // ���1���ֽ�, ����NACK�ź�
    }

    I2CSoft_Stop();                                 // �����߷���ֹͣ�ź�
    return 1;                                       // ��ȡ�ɹ������� 1

ReadBytes_ACK_Fail:                                 // �ӻ���Ӧ��ʱ�Ĵ���
    I2CSoft_Stop();                                 // ��Ҫ�������߷���ֹͣ�ź�
    return 0;                                       // ���� 0
}



/*****************************************************************************
 * ��  ����I2CSoft_WriteBytes
 * ��  �ܣ�д�����ֽ�
 * ��  ����uint8_t     _uSlaveAddr  �豸��ַ
 *         uint8_t     _uDataAddr   ���ݵ�ַ
 *         uint8_t    *_puDataBuf   ������ݵĻ����ַ
 *         uint16_t    _usNum       Ҫ��ȡ���ֽ���
 * ����ֵ��0-ʧ�ܣ�1-�ɹ�
 * �ر�أ���������δ����֤����ͬ���豸����������ͬ
*****************************************************************************/
uint8_t I2CSoft_WriteBytes(uint8_t _uSlaveAddr, uint8_t _uDataAddr, uint8_t *_puDataBuf, uint16_t _usNum)
{
    I2CSoft_Start();                        // ��ʼ�ź�

    I2CSoft_SendByte(_uSlaveAddr | 0);      // ���ʹӻ���ַ,д����, 0д1��
    if (I2CSoft_WaitAck())                  // �ȴ�ACK��ע�⣺ÿ����һ���ֽڣ���Ҫȷ�ϴӻ�Ӧ��
        goto WriteBytes_ACK_Fail;           // �ӻ���Ӧ��, ����ʧ�ܴ���(����ֹͣ�źš�����0)

    I2CSoft_SendByte(_uDataAddr);           // �������ݵ�ַ
    if (I2CSoft_WaitAck())                  // �ȴ�ACK��ע�⣺ÿ����һ���ֽڣ���Ҫȷ�ϴӻ�Ӧ��
        goto WriteBytes_ACK_Fail;           // �ӻ���Ӧ��, ����ʧ�ܴ���(����ֹͣ�źš�����0)

    for (uint16_t i = 0; i < _usNum; i++)
    {
        I2CSoft_SendByte(_puDataBuf[i]);    // ��������
        if (I2CSoft_WaitAck())              // �ȴ�ACK��ע�⣺ÿ����һ���ֽڣ���Ҫȷ�ϴӻ�Ӧ��
            goto WriteBytes_ACK_Fail;       // �ӻ���Ӧ��, ����ʧ�ܴ���(����ֹͣ�źš�����0)
        delay();
    }

    I2CSoft_Stop();                         // �����߷���ֹͣ�ź�
    return 1;                               // ��ȡ�ɹ������� 1

WriteBytes_ACK_Fail:                        // �ӻ���Ӧ��ʱ�Ĵ���
    I2CSoft_Stop();                         // ��Ҫ�������߷���ֹͣ�ź�
    return 0;                               // ���� 0
}


