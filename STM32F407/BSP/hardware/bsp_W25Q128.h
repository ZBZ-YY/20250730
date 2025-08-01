#ifndef __BSP__W25Q_H
#define __BSP__W25Q_H
/***********************************************************************************************************************************
 ** 【文件名称】  bsp_w25qxx.h
 ** 【编写人员】  魔女开发板团队
 ** 【淘    宝】  魔女开发板      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【功能描述】  定义引脚、定义全局结构体、声明全局函数
 **
 ** 【适用平台】  STM32F407 + 标准库v3.5 + keil5
 **
 ** 【硬件重点】  1- 电源引脚，并联一个100nF到GND, 不然容易通信失败；
 **               2- CS引脚，并联一个10K电阻到3.3V，可减少通信错误；
 **
 ** 【代码重点】  1- SPI通信
 **               2- W25Q256的地址, 与32,64,128等不同
 ** 　　　　　　　3- 为简化整个W25Q128的代码，已封装成3个全局函数，只用这3个函数，即可完成对其存取操作
 **                  初始化  ：  W25Q128_Init()　
 **                  读取数据：  W25Q128_ReadData( uint32_t addr, uint8_t *p, uint16_t num)　 // 数据地址，缓存指针，字节数
 **                  写入数据：  W25Q128_WriteData(uint32_t addr, uint8_t *p, uint16_t num)   // 数据地址，缓存指针，字节数
 **
 ** 【移植说明】  引脚修改：在本文件bsp_W25Q128.h中修改
 **               SPI修改 ：在h文件中和c文件的init()中需分别修改
 **
 ** 【字库使用】  特别地注意，请慎重使用芯片擦除，魔女开发板的w25q128，在存储区尾部已烧录宋体4种字号大小汉字GBK字模数据
 **               字库存放地址：0x00A00000 - 0x01000000   尽量不要写操作此区域地址
 **               具体的读取操作，可参考c文件中函数
 **
 ** 【更新记录】  2023-01-27  完善注释
 **               2022-12-29  创建文件，从F103移植至F407

***********************************************************************************************************************************/
#include <stm32f4xx.h>
#include <stdio.h>



/*****************************************************************************
 ** 引脚定义
 ** 移植时，如果使用SPI1,只需要修改这个区域
****************************************************************************/
#define  W25Q128_SPI                    SPI1
                                      
#define  W25Q128_SCK_GPIO               GPIOA  
#define  W25Q128_SCK_PIN                GPIO_Pin_5        
#define  W25Q128_SCK_PINSOURCE          GPIO_PinSource5
#define  W25Q128_SCK_AF                 GPIO_AF_SPI1
                                      
#define  W25Q128_MISO_GPIO              GPIOA 
#define  W25Q128_MISO_PIN               GPIO_Pin_6                
#define  W25Q128_MISO_PINSOURCE         GPIO_PinSource6
#define  W25Q128_MISO_AF                GPIO_AF_SPI1
                                      
#define  W25Q128_MOSI_GPIO              GPIOA 
#define  W25Q128_MOSI_PIN               GPIO_Pin_7                
#define  W25Q128_MOSI_PINSOURCE         GPIO_PinSource7
#define  W25Q128_MOSI_AF                GPIO_AF_SPI1

#define  W25Q128_CS_GPIO                GPIOC 
#define  W25Q128_CS_PIN                 GPIO_Pin_13      

#define  GBK_STORAGE_ADDR               0x00A00000    // GBK字库起始地址,魔女开发板的W25Q128已保存宋体12、16、24、32号字体



/*****************************************************************************
 ** 声明全局变量
****************************************************************************/
typedef struct
{
    uint8_t   FlagInit;                 // 初始化状态   0:失败, 1:成功
    uint8_t   FlagGBKStorage;           // GBK字库标志; 0=没有, 1=可用; 作用: 用于判断地址段的写保护, 防止字库被错误写履盖; 并可作LCD的中文输出判断
    char      type[20];                 // 型号
    uint16_t  StartupTimes;             // 记录启动次数
} xW25Q_TypeDef;
extern xW25Q_TypeDef  xW25Q128;         // 声明全局结构体, 用于记录w25qxx信息





/*****************************************************************************
 ** 声明全局函数
 ** 为统一代码以方便移植，已封装成4个对外函数，可完成对其所有存取操作
****************************************************************************/
uint8_t W25Q128_Init(void);                                                    // 初始化
void    W25Q128_ReadData(uint32_t addr, uint8_t *pData, uint16_t num);         // 读数据：addr-地址，*pData-读取后数据缓存，num-要读取的字节数
void    W25Q128_WriteData(uint32_t addr, uint8_t *pData, uint16_t num);        // 写数据：addr-地址，*pData-待写的数据缓存，num-要写入的字节数
void    W25Q128_ReadFontData(uint8_t *pFont, uint8_t size, uint8_t *fontData); // 读取字模：*pFont-汉字，size-字号，*fontData-读取到的字模点阵数据



#endif




