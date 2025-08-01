#ifndef __LCD_ILI9341_H
#define __LCD_ILI9341_H
/**==================================================================================================================
 **���ļ����ơ�  bsp_LCD_ILI9341.h
 **�����ܲ��ԡ�  2.8����ʾ��
 **==================================================================================================================
 **������ƽ̨��  STM32F407 + KEIL5.27 + 2.8����ʾ��_ILI9341
 **
 **���� �� �㡿  1-��ʾ�����룬��������󲿷�2.8��ILI9341��34����
 **              2_��ʾ�����룬�����������ı���ʾ��ʽ��ʹ������תģ����ģ���ݡ�ʹ��Flash�е���ģ���ݡ�
 **              3_���ʹ��Flash�е���ģ���ݣ���Ҫ���bsp_W25Q128.c�������У������ͷ�ļ�bsp_LCD_ILI9341.h��LCD��;
 **
 **�����¼�¼��  2024-07-15  �Ż���ģ������������ע��
 **              2024-01-25  �ӱ�׼�⹤����ֲ��HAL�⹤�̣�ʹ�üĴ�������ʵ�֣���ʹ����ֲ������
 **              2023-01-12  �������н�ģ���ı���ʾ����������ע��
 **              2022-12-30  ����������ʾ������ͼƬ��ʾ����
 **
 **����ע˵����  �����Ȩ��ħŮ�Ƽ����У��������ã�лл��
 **              https://demoboard.taobao.com
====================================================================================================================*/
#include "bsp_W25Q128.h"                      // �����ֿ�
#include "stdlib.h"

#ifdef USE_HAL_DRIVER                         // HAL�� ����
#include "stm32f4xx_hal.h"                   
#endif                                       
                                             
#ifdef USE_STDPERIPH_DRIVER                   // ��׼�� ����
#include "stm32f4xx.h"                       
#endif



/*****************************************************************************
 ** ��ֲ����
 **
****************************************************************************/
#ifdef USE_HAL_DRIVER                         // HAL�� ����
#define LCD_BL_GPIO   GPIOA                   // �����������
#define LCD_BL_PIN    GPIO_PIN_15            
#endif                                       
                                             
#ifdef USE_STDPERIPH_DRIVER                   // ��׼�� ����
#define LCD_BL_GPIO   GPIOA                   // �����������
#define LCD_BL_PIN    GPIO_Pin_15
#endif


#define LCD_WIDTH     240                     // ���Ŀ�����ص�
#define LCD_HEIGHT    320                     // ���ĸ߶����ص�




/*****************************************************************************
 ** ȫ�ֱ���
 ** (�����޸�)
****************************************************************************/
typedef struct                                // LCD��Ҫ������
{
    uint16_t width;                           // LCD ���
    uint16_t height;                          // LCD �߶�
    uint16_t id;                              // LCD ID
    uint8_t  dir;                             // ���������������ƣ�0��������1��������
    uint8_t  FlagInit;                        // ��ʼ����ɱ�־
} xLCD_TypeDef;
extern xLCD_TypeDef xLCD;                     // ����LCD��Ҫ����


/******************************* ���峣����ɫֵ *****************************/
#define      WHITE               0xFFFF       // ��ɫ
#define      BLACK               0x0000       // ��ɫ 
#define      GREY                0xF7DE       // ��ɫ 
#define      GRAY                0X8430       // ��ɫ
#define      RED                 0xF800       // �� 
#define      MAGENTA             0xF81F       // ���ɫ 
#define      GRED                0xFFE0       // ���ɫ
#define      BROWN               0XBC40       // ��ɫ
#define      BRRED               0XFC07       // �غ�ɫ
#define      GREEN               0x07E0       // �� 
#define      CYAN                0x7FFF       // ��ɫ 
#define      YELLOW              0xFFE0       // ��ɫ 
#define      LIGHTGREEN          0X841F       // ǳ��ɫ 
#define      BLUE                0x001F       // �� 
#define      GBLUE               0x07FF       // ǳ�� 1
#define      LIGHTBLUE           0X7D7C       // ǳ�� 2
#define      BLUE2               0x051F       // ǳ�� 3
#define      GRAYBLUE            0X5458       // ���� 
#define      DARKBLUE            0X01CF       // ����
#define      LGRAY               0XC618       // ǳ��ɫ,���屳��ɫ
#define      LGRAYBLUE           0XA651       // ǳ����ɫ(�м����ɫ)
#define      LBBLUE              0X2B12       // ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)



/*****************************************************************************
 ** ����ȫ�ֺ���
 ** (�����޸�)
****************************************************************************/
void LCD_Init(void);                                                                                       // ��ʼ��
void LCD_SetDir(uint8_t dir);                                                                              // ������ʾ����
void LCD_GUI(void);                                                                                        // �򵥵���ʾ����
void LCD_DisplayOn(void);                                                                                  // ����ʾ
void LCD_DisplayOff(void);                                                                                 // ����ʾ
void LCD_DrawPoint(uint16_t  x, uint16_t  y, uint16_t _color);                                             // ���㺯��
void LCD_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);                         // ��䵥ɫ
void LCD_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                         // ����
void LCD_Circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t _color);                           // ��Բ
void LCD_String(uint16_t x, uint16_t y, char *pFont, uint8_t size, uint32_t fColor, uint32_t bColor);      // ��ʾ�����ַ���(֧��Ӣ�ġ�����)
void LCD_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image);             // ͼƬ
void LCD_ShowChinese(uint8_t x, uint8_t y, uint8_t size, uint8_t indux, uint32_t fColor, uint32_t bColor); // ��ʾ����תģ���ı�
void LCD_Cross(uint16_t x, uint16_t y, uint16_t len, uint32_t fColor);                                     // ���ڴ���У׼����ʮ����
void LCD_DispFlush(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pData);        // ��ָ������������ݣ�������ͼƬ��LVGL��


#endif

