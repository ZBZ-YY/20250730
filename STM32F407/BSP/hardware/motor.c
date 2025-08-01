#include "stm32f4xx.h"
#include "motor.h"
#include "PWM.h"
#include <math.h>  // �����ѧ����֧��
// ����ת���������
#define BASE_SPEED 20     // �����ٶ�
//#define DIFFERENTIAL_GAIN 1.2f // ��������ϵ���������������ٶȲ�̶ȣ������˵ģ�
#define DIFFERENTIAL_GAIN 1.3f // ��������ϵ���������������ٶȲ�̶ȣ���������صģ�
#define MIN_DIFF 15      // ��С�ٶȲȷ��������һ�����٣�

void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // ʹ��GPIOB��GPIOEʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);
   
    // ���Ƶ������� GPIO ����ͨ���ģʽ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    

    // ���ÿ��Ƶ�� IN1~IN8 ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | 
                                  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    // ��ʼ�������������� PWM���������ڵ��ٵȣ�������ԭ�еĵ����߼� ��
    PWM_Motor_Init();
}



// ֹͣ����������Ҫ�ٶȲ�����
void Car_Stop(void) {
    // ����IN����Ϊ�͵�ƽ
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // ��������PWM���Ϊ0
    PWM_SetCompare1(0);
    PWM_SetCompare2(0);
    PWM_SetCompare3(0);
    PWM_SetCompare4(0);
}

// ǰ���������������ٶȣ�
void Car_GoForward(uint8_t speed) {
    // ��ǰ�� (PE14/PE15)
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    // ����� (PB10/PB11)
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    // �Һ��� (PB12/PB13)
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    // ��ǰ�� (PB14/PB15)
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    /*���˵ģ��ٶ�100
    // �������е��PWM�ٶ�
    PWM_SetCompare1(speed-2);  // PA0 ��ǰ��motorC
    PWM_SetCompare2(speed-2);  // PA1 �����motorA
    PWM_SetCompare3(speed-2);  // PA2 �Һ���motorB
    PWM_SetCompare4(speed);  // PA3 ��ǰ��motorD
		// �������е��PWM�ٶ�*/
		//��������ٶ�120
    PWM_SetCompare1(speed-2);  // PA0 ��ǰ��motorC
    PWM_SetCompare2(speed-2);  // PA1 �����motorA
    PWM_SetCompare3(speed-2);  // PA2 �Һ���motorB
    PWM_SetCompare4(speed);  // PA3 ��ǰ��motorD
}


void Car_TurnSmoothLeft(uint8_t angle) {
    // ����ת��Ƕȱ�����0��1��0��ʾֱ�У�1��ʾ���ת��
    float turnRatio = fabsf(angle - 90.0f) / 90.0f;
    
    // �������ֺ����ֵ��ٶȲ�
    float speedDiff = (float)BASE_SPEED * DIFFERENTIAL_GAIN * turnRatio;
    speedDiff = (speedDiff < MIN_DIFF) ? MIN_DIFF : speedDiff;
    
    // ����������ʵ���ٶ�
    uint8_t innerSpeed = (uint8_t)fmaxf(0, BASE_SPEED - speedDiff);
    uint8_t outerSpeed = (uint8_t)fminf(255, BASE_SPEED + speedDiff);
    
    // ��תʱ�����������֣����죩�����������֣�������
    // ��ǰ��ǰ�� (PE14/PE15) - �������
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    // �����ǰ�� (PB10/PB11) - ��������
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    // �Һ���ǰ�� (PB12/PB13) - ����
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    // ��ǰ��ǰ�� (PB14/PB15) - ����
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // ����PWM�ٶ� - �ڲ��ָ���������ָ���
    PWM_SetCompare1(outerSpeed);  // ��ǰ�֣�������죩
    PWM_SetCompare2(innerSpeed);  // ����֣�����������
    PWM_SetCompare3(outerSpeed);  // �Һ��֣����֣�
    PWM_SetCompare4(innerSpeed);  // ��ǰ�֣����֣�
}

void Car_TurnSmoothRight(uint8_t angle) {
    // ����ת��Ƕȱ���
    float turnRatio = fabsf(angle - 90.0f) / 90.0f;
    
    // �������ֺ����ֵ��ٶȲ�
    float speedDiff = (float)BASE_SPEED * DIFFERENTIAL_GAIN * turnRatio;
    speedDiff = (speedDiff < MIN_DIFF) ? MIN_DIFF : speedDiff;
    
    // ����������ʵ���ٶ�
    uint8_t innerSpeed = (uint8_t)fmaxf(0, BASE_SPEED - speedDiff);
    uint8_t outerSpeed = (uint8_t)fminf(255, BASE_SPEED + speedDiff);
    
    // ��תʱ�����������֣����죩�����������֣�������
    // ��ǰ��ǰ�� (PE14/PE15) - ����
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    // �����ǰ�� (PB10/PB11) - ����
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    // �Һ���ǰ�� (PB12/PB13) - ��������
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    // ��ǰ��ǰ�� (PB14/PB15) - �������
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // ����PWM�ٶ� - �ڲ��ָ���������ָ���
    PWM_SetCompare1(innerSpeed);  // ��ǰ�֣����֣�
    PWM_SetCompare2(outerSpeed);  // ����֣����֣�
    PWM_SetCompare3(innerSpeed);  // �Һ��֣�����������
    PWM_SetCompare4(outerSpeed);  // ��ǰ�֣�������죩
}

// ͨ��ת���������ɵ�׷�ٿ����У�
void Car_Steer(uint8_t baseSpeed, float steeringAngle) {
    // ����ת�������-1.0��ȫ��ת��0ֱ�У�+1.0��ȫ��ת��
    float steerRatio = (steeringAngle - 90.0f) / 90.0f;
    
    // �����ٶȲת��Խ��������Խ��
    float speedDiff = fabsf(steerRatio) * baseSpeed * DIFFERENTIAL_GAIN;
    speedDiff = fmaxf(speedDiff, MIN_DIFF);
    
    // ������������ǰ������
    GPIO_SetBits(GPIOE, GPIO_Pin_14); // ��ǰ
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_11); // ���
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_13); // �Һ�
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_14); // ��ǰ
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // ����ת�������ò���
    if(steerRatio < 0) { // ��ת
        PWM_SetCompare1(baseSpeed + speedDiff); // ��ǰ�����֣�����
        PWM_SetCompare2(baseSpeed - speedDiff); // ������֣�����
        PWM_SetCompare3(baseSpeed + speedDiff); // �Һ����֣�����
        PWM_SetCompare4(baseSpeed - speedDiff); // ��ǰ�����֣�����
    } 
    else if(steerRatio > 0) { // ��ת
        PWM_SetCompare1(baseSpeed - speedDiff); // ��ǰ�����֣�����
        PWM_SetCompare2(baseSpeed + speedDiff); // ������֣�����
        PWM_SetCompare3(baseSpeed - speedDiff); // �Һ����֣�����
        PWM_SetCompare4(baseSpeed + speedDiff); // ��ǰ�����֣�����
    }
    else { // ֱ��
        PWM_SetCompare1(baseSpeed);
        PWM_SetCompare2(baseSpeed);
        PWM_SetCompare3(baseSpeed);
        PWM_SetCompare4(baseSpeed);
    }
}
// ��ת�������������ٶȣ�
void Car_TurnRight(uint8_t speed) {
    // ��ǰ�ֺ��� (PE14/PE15)
    GPIO_SetBits(GPIOE, GPIO_Pin_15);
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    
    // �����ǰ�� (PB10/PB11)
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    
    // �Һ��ֺ��� (PB12/PB13)
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    
    // ��ǰ��ǰ�� (PB14/PB15)
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    // ����PWM�ٶ�
    PWM_SetCompare1(speed-2);  // PA0 ��ǰ��motorC
    PWM_SetCompare2(speed-2);  // PA1 �����motorA
    PWM_SetCompare3(speed-2);  // PA2 �Һ���motorB
    PWM_SetCompare4(speed);  // PA3 ��ǰ��motorD
}

// ��ת�������������ٶȣ�
void Car_TurnLeft(uint8_t speed) {
    // ��ǰ��ǰ�� (PE14/PE15)
    GPIO_SetBits(GPIOE, GPIO_Pin_14);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    
    // ����ֺ��� (PB10/PB11)
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    
    // �Һ���ǰ�� (PB12/PB13)
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    
    // ��ǰ�ֺ��� (PB14/PB15)
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    
    // ����PWM�ٶ�
    PWM_SetCompare1(speed-2);  // PA0 ��ǰ��motorC
    PWM_SetCompare2(speed-2);  // PA1 �����motorA
    PWM_SetCompare3(speed-2);  // PA2 �Һ���motorB
    PWM_SetCompare4(speed);  // PA3 ��ǰ��motorD
}

void Car_Back(uint8_t speed) {
    // ��ǰ�� (PE14/PE15)
    GPIO_SetBits(GPIOE, GPIO_Pin_15);
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    
    // ����� (PB10/PB11)
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    
    // �Һ��� (PB12/PB13)
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    
    // ��ǰ�� (PB14/PB15)
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    
    // �������е��PWM�ٶ�
    PWM_SetCompare1(speed);  // PA0 ��ǰ��
    PWM_SetCompare2(speed);  // PA1 �����
    PWM_SetCompare3(speed);  // PA2 �Һ���
    PWM_SetCompare4(speed);  // PA3 ��ǰ��
}

