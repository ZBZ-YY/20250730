#ifndef __BSP_OLED_H
#define __BSP_OLED_H 
/***********************************************************************************************************************************
 ** �������д��  ħŮ�������Ŷ�
 ** ��������¡�
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** ���ļ����ơ�  bsp_OLED.h
 **
 ** ������ƽ̨��  STM32F407 + KEIL5.31 + 0.96��4��LOED
 **
 ** ����ֲ˵����  1- ������(bsp_OLED.c��bsp_OLED.h��ͬʱ�����ڱ�׼�⡢HAL�⹤��.
 **               2- ���Ʊ����̵�OLED�ļ��У���Ŀ�깤���ļ�����;
 **               3- ��bsp_OLED.c�ļ���ӵ����̹�����;
 **               4- ���bsp_OLED.h�ļ���ͷ�ļ�·��;
 **               5- ����Ҫ���ڹ��ܵ��ļ��У�#include "bsp_OLED.h����
 **
 ** ��Ӳ���ص㡿  VCC --- 3.3V , ���鲻Ҫ��5.0V��
 **               GND --- GND
 **               SCL --- PB8   (���޸�)
 **               SDA --- PB9   (���޸�)
 **
 ** ������ص㡿  ������ʹ��ģ��I2C���������������I2C���ܡ�
 **               OLED���Դ�, ��Ÿ�ʽ����.
 **               [0]0 1 2 3 ... 127
 **               [1]0 1 2 3 ... 127
 **               [2]0 1 2 3 ... 127
 **               [3]0 1 2 3 ... 127
 **               [4]0 1 2 3 ... 127
 **               [5]0 1 2 3 ... 127
 **               [6]0 1 2 3 ... 127
 **               [7]0 1 2 3 ... 127
 **
 ** �����¼�¼��  
 **               2023-01-10   ������F407
 **
 ** ����ע˵����  ��Ȩ��ħŮ�Ƽ����У�����ѧϰ���ԣ��������ã�лл��
 **               https://demoboard.taobao.com
************************************************************************************************************************************/
#ifdef USE_STDPERIPH_DRIVER                 // ��׼��
    #include "stm32f4xx.h"                 // ��׼��ļĴ������� 
    #include "bsp_W25Q128.h"               // �ⲿFlash�洢���洢���ֿ�����
#endif

#ifdef USE_HAL_DRIVER                      // ʹ��HAL��
    #include "stm32f4xx_hal.h"             // HAL��ļĴ������� 
    #include "bsp_W25Q128.h"               // �ⲿFlash�洢���洢���ֿ�����
#endif





/*****************************************************************************
 ** ���Ŷ���
 ** ������ʹ��ģ��I2C������ʹ����ͨ���ţ����Ǳ�����I2C���ܵ����š�
****************************************************************************/
// ʹ�ñ�׼��
#ifdef   USE_STDPERIPH_DRIVER   
// SCL����
#define  OLED_SCL_GPIO   GPIOB            // SCL���ţ�ģ��I2C
#define  OLED_SCL_PIN    GPIO_Pin_8
// SDA����
#define  OLED_SDA_GPIO   GPIOB            // SDA���ţ�ģ��I2C
#define  OLED_SDA_PIN    GPIO_Pin_9
#endif

// ʹ��HAL��
#ifdef   USE_HAL_DRIVER                   // HAL��
// SCL����
#define  OLED_SCL_GPIO   GPIOB            // SCL���ţ�ģ��I2C
#define  OLED_SCL_PIN    GPIO_PIN_8
// SDA����       
#define  OLED_SDA_GPIO   GPIOB            // SDA���ţ�ģ��I2C
#define  OLED_SDA_PIN    GPIO_PIN_9
#endif





/*****************************************************************************
 ** ����ȫ�ֺ���
****************************************************************************/
void OLED_Init(void);                                                   // ��ʼ��
void OLED_Refresh(void);                                                // �����Դ浽OLED
void OLED_DisplayTurn(uint8_t i);                                       // ��ʾ����ˮƽ��ת
void OLED_Display(uint8_t sw);                                          // ��ʾ��Դ����
void OLED_Clear(void);                                                  // ����
void OLED_Line(uint16_t  x1, uint16_t  y1, uint16_t  x2, uint16_t  y2); // ����
void OLED_Circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);        // ��Բ
void OLED_String(uint16_t x, uint16_t y, char *pFont, uint8_t size);    // ��ʾ��Ӣ���ַ���
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1);   // ��ʾ����(��ģ������font.h)
void OLED_ShowNumber(uint16_t x, uint16_t y, uint32_t num, uint8_t size);
#endif

