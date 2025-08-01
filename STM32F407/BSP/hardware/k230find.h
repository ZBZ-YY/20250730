#ifndef __K230find_H
#define __K230find_H

#include "stm32f4xx.h"
#include <math.h>

void k230find_Init(void);
void k230find_Update(void);
void START_GO(void);
void TargetTracking_ChassisControl1(void);
void TargetTracking_ChassisControl2(void);
#endif
