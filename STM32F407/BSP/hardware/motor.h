#ifndef __MOTOR_H
#define __MOTOR_H
void Motor_Init(void);
void Car_Stop(void);
void Car_GoForward(uint8_t speed);
void Car_TurnRight(uint8_t speed);
void Car_TurnLeft(uint8_t speed);
void Car_Back(uint8_t speed);
void Car_TurnSmoothLeft(uint8_t angle);
void Car_TurnSmoothRight(uint8_t angle);
#endif

