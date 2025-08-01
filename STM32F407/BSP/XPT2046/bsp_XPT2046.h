#ifndef  __BSP_XPT2046_H
#define  __BSP_XPT2046_H
/***********************************************************************************************************************************
 ** �������д��  ħŮ�������Ŷ�
 ** ��������¡�
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** ���ļ����ơ�  bsp_XPT2046.h
 **
 ** ���ļ����ܡ�  �������š�����ȫ�ֽṹ�塢����ȫ�ֺ���
 **               ���ļ��򻯵�XPT2046�ĳ�ʼ��; ��ʼ������ú�������ʹ�ã�
 **
 ** ������ƽ̨��  STM32F407 + keil5 
 **
 ** ����ֲ˵����  1- ���Ʊ����̵�LCD_ILI9341��W25Q128��XPT2046�ļ��У���Ŀ�깤���ļ�����;
 **               2- ���ͷ�ļ�·���� Keil > Option > C/C++ > Include Paths, �������3��Ŀ���ļ��е�Ŀ¼·��;
 **               3- ���C�ļ�������: Keil > ��๤�̹�������˫��Ŀ���ļ��� > ѡ�� bsp_LCD_ILI9341.c��bsp_W25Q128.c��bsp_XPT2046;
 **               4- ����ļ�����:    #include "bsp_XPT2046.h"�����ĸ��ļ�����Ҫʹ���亯�����ܣ�����������ļ������������;
 **
 ** �� CubeMX ��  1- ����ʹ��CubeMX��������
 **               2- ��bsp_XPT2046.h�ļ����޸�ʹ�õ�����
 **               
 ** ������ʹ�á�  1- ���ڴ�����״̬��⡢�����ȡ����ʾ��ͨ�����������Ѱѻ��������ú�ֱ���ˡ������������˷����������ף��Ѿ���������~~��
 **               2- ��ʾ��ͨ��������㣬�Ѱѻ���������д���ˣ��û����������������������ƣ��磺ͨ�����㺯����ư�ť����������ͨ��������������ƴ�����
 **               3- ���������������л�����Ҫ�Ŀؼ�����������ƿؼ����¼���������ֲʹ��LVGL; LVGL���ŵ㣺���ɻ��ƿ��ơ��¼����������˽����ĵײ����; LVGL��ȱ�㣺ռ�ô�����Դ;
 **               4- ������Ӧ�ã��ײ�������Ĳ������������ǰ��̨��FreeRTOS��LVGL��
 **                  �� ʼ ��: XPT2046_Init(xLCD.width, xLCD.height, xLCD.dir);   // ��ʼ������ȣ��߶ȣ�����
 **                  �����жϣ�XPT2046_IsPressed();                               // ��鴥������ǰ�Ƿ񱻰���; ���أ�0-δ���¡�1-����; ������£����Ե�����������������ȡ����;
 **                  ��ȡ���꣺XPT2046_GetX();                                    // ��ȡ���µ�λ�õ����꣺X; ����ֵ���ͣ�uint16_t
 **                            XPT2046_GetX();                                    // ��ȡ���µ�λ�õ����꣺Y; ����ֵ���ͣ�uint16_t
 **               5- ������ִ���λ�ò�׼ȷ, �����µ�λ���뻭���λ���������ԣ����Խ�������У׼����¼���б�ʾ�����ô������ַ���"XPT2046", ���򼴿ɽ�������У׼��
 **               6- �ر�أ����� #include "bsp_W25Q128.h"����Ϊ�������¼�˵�������У׼����; ���ʹ�ù̶�����ʾ�����̶�����ʾ���򣬿���ֱ�Ӱ����洢��������;
 **
 ** �����¼�¼��
 **
 ** ����ע˵����  ��Ȩ��ħŮ�Ƽ����У�����ѧϰ���ԣ��������ã�лл��
 **               https://demoboard.taobao.com
 
************************************************************************************************************************************/
#include "bsp_lcd_ili9341.h"
#include "stdio.h"


#ifdef USE_HAL_DRIVER                                        // HAL�� ����
#include "stm32f4xx_hal.h"
#endif

#ifdef USE_STDPERIPH_DRIVER                                  // ��׼�� ����
#include "stm32f4xx.h"
#endif




/*****************************************************************************
 ** ���ų趨��
 ** ��ע��ʹ�õ���ģ��SPI, INT����ʹ��״̬��ѯ, ��ʹ���ж�;
****************************************************************************/
#ifdef USE_HAL_DRIVER                                        // HAL�� ����
#define    XPT2046_IRQ_GPIO         GPIOE                    // �����ź�ָʾ����(��ʹ���ж�) 
#define    XPT2046_IRQ_PIN          GPIO_PIN_4               
#define    XPT2046_IRQ_PORT_CLK     RCC_AHB1Periph_GPIOE     
                                                             
#define    XPT2046_CS_GPIO          GPIOD                    // ģ��SPI_CS
#define    XPT2046_CS_PIN           GPIO_PIN_13              
#define    XPT2046_CS_PORT_CLK      RCC_AHB1Periph_GPIOD     
                                                             
#define    XPT2046_CLK_GPIO         GPIOE                    // ģ��SPI_CLK
#define    XPT2046_CLK_PIN          GPIO_PIN_0               
#define    XPT2046_CLK_PORT_CLK     RCC_AHB1Periph_GPIOE     
                                                             
#define    XPT2046_MOSI_GPIO        GPIOE                    // ģ��SPI_MOSI
#define    XPT2046_MOSI_PIN         GPIO_PIN_2               
#define    XPT2046_MOSI_PORT_CLK    RCC_AHB1Periph_GPIOE     
                                                             
#define    XPT2046_MISO_GPIO        GPIOE                    // ģ��SPI_MISO
#define    XPT2046_MISO_PIN         GPIO_PIN_3  
#define    XPT2046_MISO_PORT_CLK    RCC_AHB1Periph_GPIOE
#endif

#ifdef USE_STDPERIPH_DRIVER                                  // ��׼�� ����
#define    XPT2046_IRQ_GPIO         GPIOE                    // �����ź�ָʾ����(��ʹ���ж�) 
#define    XPT2046_IRQ_PIN          GPIO_Pin_4               
#define    XPT2046_IRQ_PORT_CLK     RCC_AHB1Periph_GPIOE     
                                                             
#define    XPT2046_CS_GPIO          GPIOD                    // ģ��SPI_CS
#define    XPT2046_CS_PIN           GPIO_Pin_13              
#define    XPT2046_CS_PORT_CLK      RCC_AHB1Periph_GPIOD     
                                                             
#define    XPT2046_CLK_GPIO         GPIOE                    // ģ��SPI_CLK
#define    XPT2046_CLK_PIN          GPIO_Pin_0               
#define    XPT2046_CLK_PORT_CLK     RCC_AHB1Periph_GPIOE     
                                                             
#define    XPT2046_MOSI_GPIO        GPIOE                    // ģ��SPI_MOSI
#define    XPT2046_MOSI_PIN         GPIO_Pin_2               
#define    XPT2046_MOSI_PORT_CLK    RCC_AHB1Periph_GPIOE     
                                                             
#define    XPT2046_MISO_GPIO        GPIOE                    // ģ��SPI_MISO
#define    XPT2046_MISO_PIN         GPIO_Pin_3  
#define    XPT2046_MISO_PORT_CLK    RCC_AHB1Periph_GPIOE
#endif



/*****************************************************************************
 ** ����ȫ�ֺ�������                 
 *****************************************************************************/
// ��ʼ��
void      XPT2046_Init (uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir ); // ��ʼ��(����, У׼���);  ����: ��ʾ������ʾ����, ȡֵ��Χ: 1,2,3,4
// ����У׼
uint8_t   XPT2046_ReCalibration(void);                                        // ������������У׼; ���������겻��ʱ������һ�α��������ñʼ����ε��ʮ�ֹ���߼��ɡ�
// ������⡢��ȡ����; 
uint8_t   XPT2046_IsPressed(void);                                            // �жϴ���������״̬; ����:0-�ͷš�1-����; ��ֲLVGLʱ����lv_port_indev.c�ļ���touchpad_is_pressed()��������ô˺�������;
uint16_t  XPT2046_GetX(void);                                                 // ��ȡ���µ�X����ֵ; ��ֲLVGLʱ����lv_port_indev.c�ļ���touchpad_get_xy()���ñ��������ɻ��(*X)ֵ;
uint16_t  XPT2046_GetY(void);                                                 // ��ȡ���µ�Y����ֵ; ��ֲLVGLʱ����lv_port_indev.c�ļ���touchpad_get_xy()���ñ��������ɻ��(*Y)ֵ;  
// ���ʾ���õĺ���; LVGL���ò���������ɾ��, ��Ӱ��ײ�
void      XPT2046_Cmd(uint8_t status);                                        // ������⿪��; �Ǳ�Ҫ��; ��ʼ����Ĭ���ǿ�����; �����ֶ��رգ���ͣ�����ʵ��һЩ������Ȼ�����ֶ�����
void      XPT2046_TouchDown(void);                                            // �հ׺���, ���ð���ʱ�Ĵ���
void      XPT2046_TouchUp(void);                                              // �հ׺���, �����ͷ�ʱ�Ĵ���   




#endif 

