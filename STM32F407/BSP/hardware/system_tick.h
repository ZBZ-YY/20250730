#ifndef __SYSTEM_TICK_H
#define __SYSTEM_TICK_H

#include <stdint.h>

void SystemTick_Init(void);
uint32_t GetSystemTick(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

#endif
