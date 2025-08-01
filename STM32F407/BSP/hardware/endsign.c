#include "stm32f4xx.h"
#include "endsign.h"

/*���ų�ʼ��*/
void Endsign_Init(void)
{
    /*����ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
															
	
	/*GPIO��ʼ��*/
	GPIO_InitTypeDef GPIO_InitStructure;					
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          // ͨ�����ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // �������		
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
