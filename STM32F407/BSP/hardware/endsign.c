#include "stm32f4xx.h"
#include "endsign.h"

/*引脚初始化*/
void Endsign_Init(void)
{
    /*开启时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
															
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;					
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          // 通用输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // 推挽输出		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);		
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);	
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
															
}
void Endsign_open(void){	
	GPIO_SetBits(GPIOB, GPIO_Pin_0);	
	GPIO_SetBits(GPIOB, GPIO_Pin_1);	
}

void Endsign_off(void){	
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);	
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);	
}
