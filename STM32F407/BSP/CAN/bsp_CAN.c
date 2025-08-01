#include "bsp_CAN.h"
#include "string.h"





/******************************************************************************
 * ��  ���� CAN1_Config
 * ��  �ܣ� ����CAN
 * ��  ���� ��
 * ����ֵ�� ��
 * ��  ע�� 
******************************************************************************/
void CAN1_Config(void)
{
    // 1-��������CAN����Ľṹ��
    GPIO_InitTypeDef GPIO_InitStructure;                          // GPIO���ýṹ��
    CAN_InitTypeDef  CAN_InitStructure;                           // CAN���ýṹ��
    CAN_FilterInitTypeDef CAN_FilterInitTypeStruct;               // ɸѡ�����ýṹ��
   
    // 2-ʹ��ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);          // ʹ��CAN1ʱ��

#if CAN_GPIO_REMAP                                                // ��ֵ��bsp_CAN.h�ļ��ж��壬�����ж�ʹ��PA11+PA12������ʹ��PB8+PB9
    // 2-ʹ��ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);         // ʹ��GPIOAʱ��
    // 3-��ʼ��CAN�����GPIO:CAN_TX_PB9
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 ;                  // CAN_TX_PB9
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                 // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;            // �����ٶ�
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;             //
    GPIO_Init(GPIOB, &GPIO_InitStructure);                        // ��ʼ������
    // 4-��ʼ��CAN�����GPIO:CAN_RX_PB8
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 ;                  // CAN_RX_PB8
    GPIO_Init(GPIOB, &GPIO_InitStructure);                        // ��ʼ������
    // 5-�������Ÿ��ù���
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
#else
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);         // ʹ��GPIOAʱ��
    // 3-��ʼ��CAN�����GPIO:CAN_TX_PA12
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 ;                 // CAN_TX_PA12
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                 // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;             // �����ٶ�
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                 //
    GPIO_Init(GPIOA, &GPIO_InitStructure);                        // ��ʼ������
    // 4-��ʼ��CAN�����GPIO:CAN_RX_PA11
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11 ;                 // CAN_RX_PA11
    GPIO_Init(GPIOA, &GPIO_InitStructure);                        // ��ʼ������
    // 5-�������Ÿ��ù���
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
#endif
  
    // 6-��ʼ��CAN����
    CAN_InitStructure.CAN_ABOM = ENABLE;                          // �˳����߹���; DISABLE-���������; ENABLE-Ӳ���Զ�����
    CAN_InitStructure.CAN_AWUM = ENABLE;                          // �Զ����ѹ���; DISABLE-���������; ENABLE-Ӳ���Զ�����
    CAN_InitStructure.CAN_NART = ENABLE;                          // ��ֹ�Զ��ش�; DISABLE-���ͱ���ʧ��ʱһֱ�Զ��ش�ֱ�����ͳɹ�; ENABLE-����ֻ������1�Σ����ܷ��͵Ľ�����
    CAN_InitStructure.CAN_RFLM = DISABLE;                         // ����FIFO����; DISABLE-���������ʱ��FIFO�ı���δ���������µ��ı��Ľ����Ǿɱ���; ENABLE-�±��Ĳ��ܸ��Ǿɱ��ģ�ֱ������
    CAN_InitStructure.CAN_TTCM = DISABLE;                         // ʱ�䴥��ģʽ��DISABLE-��ֹʱ�䴥��ͨ��ģʽ; ENABLE-����ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_TXFP = ENABLE;                          // ���͵����ȼ�; DISABLE-�������ȼ��ɱ��ĵı�ʶ��������; ENABLE-�������ȼ������Ĵ���������Ⱥ�˳����
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal ;                // ����ģʽ����Normal=����, LoopBack=�ػ�, Silent=��Ĭ, Silent_LoopBack=��Ĭ�ػ�
 
    // 7-�����ʡ�λʱ������_500K
    CAN_InitStructure.CAN_Prescaler = 6;                          // ��Ƶϵ��, ֱ����дҪ�ķ�Ƶֵ���������Զ���1
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;                      // ʱ���1
    CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;                     // ʱ���2
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;                      // ʱ���3
    CAN_Init(CAN1, &CAN_InitStructure);                           // ��ʼ��CAN��������������������д��Ĵ�����
    
    // 8-���ù����������򣺽�����������֡
    CAN_FilterInitTypeStruct.CAN_FilterNumber = 1;                                                                       // ָ��ʹ���ĸ�������; F103ϵ�п�ѡ:0~13, F105��F107ϵ�п�ѡ:0~27
    CAN_FilterInitTypeStruct.CAN_FilterMode   = CAN_FilterMode_IdMask ;                                                  // ����ģʽ��IDMask=0=����λģʽ��IdList=1=�б�ģʽ
    CAN_FilterInitTypeStruct.CAN_FilterScale  = CAN_FilterScale_32bit ;                                                  // ������λ��
    CAN_FilterInitTypeStruct.CAN_FilterIdHigh = (((uint32_t)0x000 << 3) & 0xFFFF0000) >> 16;                             // ����ģʽʱ����֤��ĸ�16λֵ
    CAN_FilterInitTypeStruct.CAN_FilterIdLow  = (((uint32_t)0x000 << 3 | CAN_Id_Standard | CAN_RTR_Data) & 0xFFFF);      // ����ģʽʱ����֤��ĵ�16λֵ
    CAN_FilterInitTypeStruct.CAN_FilterMaskIdHigh = ((uint32_t)0x000 << 3) & 0xFFFF0000 >> 16;                           // ���������α�ʶ���ĸ�16λ; �ص㽨�飬�ö�������������ֵ������������; ���������λΪ0ʱ����ʾ���ù������ϵ�֡ID���λ��ɶֵ����ͨ��; ��λΪ1ʱ��֡ID�����λ����������֤���λ��ֵ��ͬ������ͨ��
    CAN_FilterInitTypeStruct.CAN_FilterMaskIdLow  = (((uint32_t)0x000 << 3 | CAN_Id_Standard | CAN_RTR_Data) & 0xFFFF);  // ���������α�ʶ���ĵ�16λ; ���ϣ�����ߵ�λ�Ĵ�����������Ϊ0x0000����ʾ����������֤��Աȣ�����֡ID��ͨ�����������б�������; ���������Ϊ0xFFFF, ��ʾ֡IDֵ��ÿһλ������֤����ͬ���Ž��ձ���;
    CAN_FilterInitTypeStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;                                                // FIFO������������������������յ������ݣ���ŵ�FIFO0; CAN������������FIFO��ÿ��FIFO����3���������䣬������ȫ��Ӳ������; ���յ��ı��ģ����Զ���ŵ�������,��ȡʱ���Զ����������յ��ı���,���ù����ĸ������)
    CAN_FilterInitTypeStruct.CAN_FilterActivation = ENABLE;                                                              // ʹ�ܹ�����
    CAN_FilterInit(&CAN_FilterInitTypeStruct);
    
    printf("CAN1 ��ʼ������          �������;���ռ�����...\r");
}
    


// ����CAN����
void CAN_SendData(void)
{
    // ׼��CANҪ���͵�����
    static char strTest[8] = "CAN_Test";
    // ���÷��͵�֡����
    static CanTxMsg xCAN_TX;
    xCAN_TX.IDE    = CAN_Id_Standard;                                           // ֡��ʽ
    xCAN_TX.ExtId  = 0xBB;                                                      // ��ʶ��
    xCAN_TX.RTR    = CAN_RTR_Data;                                              // ֡����
    xCAN_TX.DLC    = 8;                                                         // Ҫ���͵����ݳ���, ע�⣺CAN��֡��Ч�������ֵ��8�ֽڣ�
    for (uint8_t i = 0; i < 8; i++)                                             // ����Ҫ���͵����ݣ���䵽�ṹ����
        xCAN_TX.Data[i] = strTest[i];
    // ����
    CAN_Transmit(CAN1, &xCAN_TX);                                               // ����CAN����
}



// ����Ƿ���յ����±���; ����ǣ�����ӡ���������������۲�
void CAN_CheckReceived(void)
{
    static uint16_t  canReceivedCNT = 1;                                        // ���ڼ����ѽ����˶���֡����; �Ǳ�Ҫ;
    static CanRxMsg  xCAN_RX;
    static char      canString[9];
     
    static char strCAN[100];
    static uint16_t timeCAN = 0;        // ����CAN��LCD��ʾ������ʱ
    static uint8_t lcdCAN = 0;

    if (CAN_MessagePending(CAN1, CAN_FIFO0))                                    // ����յ���CAN����
    {
        memset((void *)&xCAN_RX, 0x00, sizeof(xCAN_RX));                        // �ṹ����������0
        CAN_Receive(CAN1, CAN_FIFO0, &xCAN_RX);                                 // �ѽ��յ���CAN���ģ����Ͳ����뵽�ṹ����

        printf("\r****** CAN ���յ���%d֡������ ******", canReceivedCNT++);     // ׼����CAN֡���ģ���ϸ��������������������۲����
        printf("\r ֡���ͣ�%s",   xCAN_RX.RTR ? "ң��֡" : "����֡");           // ֡����
        printf("\r ֡��ʽ��%s",   xCAN_RX.IDE ? "��չ֡" : "��׼֡");           // ֡��ʽ
        printf("\r ��ʶ����0x%X", xCAN_RX.IDE ? xCAN_RX.ExtId : xCAN_RX.StdId); // ��չ֡ID
        printf("\r �ֽ�����%d",   xCAN_RX.DLC);                                 // �ֽ���
        printf("\r ������ƥ����ţ�%d", xCAN_RX.FMI);                           // ������ƥ�����; �͹�������ţ��ǲ�һ���ġ���ţ��ӹ�����0��ʼ��ÿ��16λ�������+2, 32λ��+1, û�б�ʹ�õĹ�������Ĭ����16λ��+2;

        printf("\r ��ʾ����(16����)��");                                        // 16���Ʒ�ʽ��ʾ���ݣ�����۲���ʵ����
        for (uint8_t i = 0; i < 8; i++)
            printf(" 0x%X ", xCAN_RX.Data[i]);

        memcpy(canString, xCAN_RX.Data, 8);
        printf("\r ��ʾ����(ASCII) �� %s\r", canString);                        // ASCII��ʽ��ʾ���ݣ�����۲��ַ�������; xCAN.ReceivedBuf[9]������9���ֽڣ�����Ϊ�˷������ַ�ʽ�������9�ֽ�Ϊ:'\0'

        if (strstr((char *)xCAN_RX.Data, "CAN_Test"))                           // �ж��Ƿ���Թ��߷�����������
        {
            static char strTemp[8] = "CAN_OK";                                  // ����ָ������
            static CanTxMsg xCAN_TX;
            xCAN_TX.IDE    = CAN_Id_Extended;                                   // ֡��ʽ
            xCAN_TX.ExtId  = 0xBB;                                              // ��ʶ��
            xCAN_TX.RTR    = CAN_RTR_Data;                                      // ֡����
            xCAN_TX.DLC    = 8;                                                 // Ҫ���͵����ݳ���, ע�⣺CAN��֡��Ч�������ֵ��8�ֽڣ�
            for (uint8_t i = 0; i < 8; i++)                                     // ����Ҫ���͵����ݣ���䵽�ṹ����
                xCAN_TX.Data[i] = strTemp[i];
            CAN_Transmit(CAN1, &xCAN_TX);                                       // ����CAN����
        }           

        timeCAN = 0;
        lcdCAN = 1;
        LCD_Fill(120, 146, 239, 204, BLACK);                                    // ��������ʾ��LCD�Ϸ���۲�
        LCD_String(125, 149, (char *)canString, 16, GREEN, BLACK);
        sprintf(strCAN, "%X %X %X %X", xCAN_RX.Data[0], xCAN_RX.Data[1], xCAN_RX.Data[2], xCAN_RX.Data[3]);
        LCD_String(125, 168, strCAN, 16, WHITE, BLACK);
        sprintf(strCAN, "%X %X %X %X", xCAN_RX.Data[4], xCAN_RX.Data[5], xCAN_RX.Data[6], xCAN_RX.Data[7]);
        LCD_String(125, 185, strCAN, 16, WHITE, BLACK);
    }
    else
    {
        if (++timeCAN > 5000 && lcdCAN == 1)                                    // ����N��û�յ������ݣ����LCD�ϵ���ʾ����
        {
            LCD_Fill(120, 146, 239, 204, BLACK);
            lcdCAN = 0;
        }
    }    
}


