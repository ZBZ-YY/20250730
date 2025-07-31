#include "ti_msp_dl_config.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
unsigned short Anolog[8]={0};
unsigned short white[8]={1800,1800,1800,1800,1800,1800,1800,1800};
unsigned short black[8]={300,300,300,300,300,300,300,300};
unsigned short Normal[8];
unsigned char rx_buff[256]={0};
/********************************************No_Mcu_Demo*******************************************/
/*****************芯片型号 MSPM0G3507 主频80Mhz ***************************************************/
/*****************引脚 AD0:PB0 AD1:PB1 AD2:PB2  ***************************************************/
/*****************OUT PA27*************************************************************************/
/********************************************No_Mcu_Demo*******************************************/
int main(void)
{
	//初始化
		No_MCU_Sensor sensor;
		unsigned char Digtal;
    SYSCFG_DL_init();
		
	//初始化传感器，不带黑白值
		No_MCU_Ganv_Sensor_Init_Frist(&sensor);
		No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
		Get_Anolog_Value(&sensor,Anolog);
	//此时打印的ADC的值，可用通过这个ADC作为黑白值的校准
	//也可以自己写按键逻辑完成一键校准功能
		//sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
		//ring((char *)rx_buff);
		//delay_ms(100);
		memset(rx_buff,0,256);
	//得到黑白校准值之后，初始化传感器
		No_MCU_Ganv_Sensor_Init(&sensor,white,black);
	
		//delay_ms(100);
	
		while (1) {
			//无时基传感器常规任务，包含模拟量，数字量，归一化量
			No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
			//有时基传感器常规任务，包含模拟量，数字量，归一化量
//			No_Mcu_Ganv_Sensor_Task_With_tick(&sensor)
			//获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
			Digtal=Get_Digtal_For_User(&sensor);
			//sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
			//uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			
			//获取传感器模拟量结果(有黑白值初始化后返回1 没有返回 0)
			if(Get_Anolog_Value(&sensor,Anolog)){
			//sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
			//uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			}
			
			//获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个值！！有黑白值初始化后返回1 没有返回 0)
			if(Get_Normalize_For_User(&sensor,Normal)){
			//sprintf((char *)rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
			//uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			}
			//经典版理论性能1khz，只需要delay1ms，青春版100hz，需要delay10ms，否则不能正常使用
			//delay_ms(1);
		}
}