#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stm32f4xx.h"

#define SERIAL_RX_PACKET_SIZE 3  // ʵ�����ݣ����X,���Y,����

extern uint8_t Serial_RxPacket[];
extern volatile uint8_t Serial_RxFlag;

void Serial_Init(void);

uint8_t Serial_GetRxFlag(void);  // ��ȡ������ɱ�־
#endif
