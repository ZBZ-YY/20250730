#include "stm32f4xx.h"
#include "PWM.h"
#include "motor.h"
#include "hongwai.h"
#include "bsp_OLED.h"
#include "serial.h"
#include "k230.h"
#include "k230find.h"
#include "system_tick.h"
#include "endsign.h"

extern volatile uint8_t Serial_RxFlag;
extern uint8_t targetDetected; // ����Ŀ�����־
volatile uint32_t lastTargetUpdateTime = 0;

// ����ȫ�ֱ���
uint32_t avoidObstacleStartTime = 0; // ���Ͽ�ʼʱ��
uint8_t isAvoidingObstacle = 0;      // �Ƿ����ڱ���

// ������ʼ������ - ��������Ű���
void Key_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. ʹ��GPIODʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    // 2. ����PD12Ϊ����ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        // ����ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // �����������Ȼ�������룩
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // ����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // ����������������ģ���Դ���
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

// ������⺯�� - ��������Ű���
uint8_t Key_GetState(void) {
    // ֱ�Ӷ�ȡOUT����״̬
    return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12);
}

int main(void)
{
	
    // ��ʼ��ϵͳ
    Endsign_Init();
    SystemTick_Init(); // ���ϵͳ�δ�ʱ����ʼ��
    lastTargetUpdateTime = GetSystemTick(); // ������г�ʼ��
    Motor_Init(); // �����ʼ��
    EXTI_INFRAREDAVOID1_Config(); // ����1�ж�
    EXTI_INFRAREDAVOID2_Config(); // ����2�ж�
    EXTI_INFRAREDAVOID3_Config(); // ����3�ж�
    EXTI_INFRAREDAVOID4_Config(); // ����4�ж� 
    
    // ��ʼ������
    Key_Init();
    
    // ��ʼ������״̬����
    avoidObstacleStartTime = 0;
    isAvoidingObstacle = 0;
    
    // �ȴ��������� - ȷ��ϵͳ��ͣ������
    while (1) {
        // ��������ʱOUT����Ϊ�ߵ�ƽ
        if (Key_GetState() == 0) {
            // �����ʱȷ�ϰ���
            Delay_ms(50);
            if (Key_GetState() == 0) {
                break; // ȷ�ϰ�������
            }
        }
        // ��Ӷ�����ʱ����CPUռ��
        Delay_ms(10);
    }
    
	  //�����Ƽ��˶������߼�
    START_GO();
    
    // ��ʼ��K230����ϵͳ
    k230find_Init();
    

  // ��ѭ��
while(1)
{

    k230find_Update();
    
    uint32_t currentTime = GetSystemTick();
    
    // 1. ������״̬�Ƿ���Ҫ����
    if (isAvoidingObstacle && (currentTime - avoidObstacleStartTime) >= 500) {
        isAvoidingObstacle = 0; // 0.5����Ͻ���
    }
    
    // 2. һ���Զ�ȡ���д�����״̬����ʹ���ĸ���������
    uint32_t sensorData = GPIO_ReadInputData(GPIOD);
    uint8_t right_clear = (sensorData & GPIO_Pin_8) != 0;  // ��
    uint8_t forwardright_clear = (sensorData & GPIO_Pin_7) != 0; // ��ǰ
    uint8_t left_clear = (sensorData & GPIO_Pin_9) != 0;   // ��
    uint8_t forwardleft_clear = (sensorData & GPIO_Pin_6) != 0;  // ��ǰ
    
    // 3. �ϰ����
    uint8_t obstacleDetected = 0;
    uint8_t turnRightFlag = 0;  // ��ת��־
    uint8_t turnLeftFlag = 0;   // ��ת��־
    uint8_t stopFlag = 0;       // ֹͣ��־
    
    // �����ϰ������������ĸ���������
    int obstacleCount = !right_clear + !forwardright_clear + 
                        !left_clear + !forwardleft_clear;
    
    if (obstacleCount > 0) {
        obstacleDetected = 1;
        
        // ��¼���Ͽ�ʼʱ��
        if (!isAvoidingObstacle) {
            isAvoidingObstacle = 1;
            avoidObstacleStartTime = currentTime;
        }
        
        // ����1: �����ĸ�����������⵽�ϰ� - ֹͣ
        if (obstacleCount == 4) {
            stopFlag = 1;
        }
        // ����2: ������������⵽�ϰ�
        else if (obstacleCount == 3) {
            // �������������ϰ�����
            int leftObstacleCount = !left_clear + !forwardleft_clear;
            int rightObstacleCount = !right_clear + !forwardright_clear;
            
            if (leftObstacleCount > rightObstacleCount) {
                // ����ϰ��� -> ����ת
                turnRightFlag = 1;
            } else {
                // �Ҳ��ϰ��� -> ����ת
                turnLeftFlag = 1;
            }
        }
        // ����3: ������������⵽�ϰ�
        else if (obstacleCount == 2) {
            // 1. ��ǰ�������ϰ� -> ��ת
            if (!forwardleft_clear && !left_clear) {
                turnRightFlag = 1;
            }
            // 2. ��ǰ���Ҷ����ϰ� -> ��ת
            else if (!forwardright_clear && !right_clear) {
                turnLeftFlag = 1;
            }
            // 3. ��ǰ���Ҷ����ϰ� -> ��ת
            else if (!forwardleft_clear && !right_clear) {
                turnLeftFlag = 1;
            }
            // 4. ��ǰ�������ϰ� -> ��ת
            else if (!forwardright_clear && !left_clear) {
                turnRightFlag = 1;
            }
            // 5. ��ǰ����ǰ�����ϰ� -> ���ݷ������ȼ�����
            else if (!forwardleft_clear && !forwardright_clear) {
                // ǰ�����ϰ����������Ҵ�����״̬����
                if (!left_clear) {
                    turnRightFlag = 1;  // ������ϰ�����ת
                } else if (!right_clear) {
                    turnLeftFlag = 1;   // �ұ����ϰ�����ת
                } else {
                    // ���Ҷ�û���ϰ������ѡһ����������ѡ����ת��
                    turnRightFlag = 1;
                }
            }
            // 6. �����Ҳ඼���ϰ� -> ��ת
            else if (!left_clear && !right_clear) {
                turnLeftFlag = 1;
            }
            // 7. ����δ��ȷ��� - ��ȫֹͣ
            else {
                stopFlag = 1;
            }
        }
        // ����4: һ����������⵽�ϰ�
        else if (obstacleCount == 1) {
            // ������ǰ���ϰ� -> ��ת
            if (!left_clear || !forwardleft_clear) {
                turnRightFlag = 1;
            }
            // �Ҳ����ǰ���ϰ� -> ��ת
            else if (!right_clear || !forwardright_clear) {
                turnLeftFlag = 1;
            }
        }
    }
    
    // 4. ִ�б��϶���
    if (obstacleDetected) {
        if (stopFlag) {
            Car_Stop();
        }
        else if (turnRightFlag) {

            Car_TurnRight(80); // 1 = ��ת
        }
        else if (turnLeftFlag) {
            Car_TurnLeft(80); // 0 = ��ת
        }
        else {
            Car_Stop();
        }
    }
    else {
        // ���ϰ� - ���ñ��ϱ�־
        avoidRightActive = 0;
        avoidLeftActive = 0;
        avoidForwardRightActive = 0;
        avoidForwardLeftActive = 0;
        Car_GoForward(20);
        
        if (targetDetected && !isAvoidingObstacle) {
            TargetTracking_ChassisControl1();
//			Car_Stop();
        } 

    }
}
}
