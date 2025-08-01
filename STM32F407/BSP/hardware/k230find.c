#include "stm32f4xx.h"
#include "Serial.h"
#include "k230.h"
#include "delay.h"
#include <math.h>
#include "motor.h"
#include "system_tick.h"  
#include "endsign.h"
#include "PWM.h"
// ����ת�����
#define BASE_SPEED 30       // �����ٶ�ֵ
#define DIFF_GAIN 0.8f      // ��������ϵ��(ԭ1.2f)
#define MIN_DIFF 10         // ��С��С�ٶȲ�(ԭ15)
#define DEAD_ZONE 10.0f      // �����Ƕȷ�Χ(��)
// ����ǶȲ���
float Angle1 = 135.0;      // ˮƽ�����ǰ�Ƕ�
float Angle2 = 90.0;      // ��ֱ�����ǰ�Ƕ�
float TargetAngle1 = 135.0; // ˮƽ���Ŀ��Ƕ�
float TargetAngle2 = 90.0; // ��ֱ���Ŀ��Ƕ�
uint8_t moveFlag = 0;     // �ƶ���־��0=ֹͣ��1=�ƶ��У�
uint8_t isBackingUp = 0;      // ����״̬��־

#define DEADZONE_ANGLE 0.3f // ����������ֵ����λ���ȣ�
#define TARGET_LOST_TIMEOUT 1000  // 1�볬ʱ(��λms)
extern volatile uint32_t lastTargetUpdateTime; // ����յ�Ŀ���ʱ���

uint8_t currentDistance = 0;  // �洢��ǰĿ�����
uint8_t targetDetected = 0;   // Ŀ�����־ (0=��Ŀ�꣬1=��Ŀ��)
uint32_t closeStartTime = 0;  // ��¼��ʼС��80cm��ʱ��
uint8_t endSignTriggered = 0; // ��ǽ����ź��Ƿ��Ѵ���

// ������Ѳ��״̬����
uint8_t patrolMode = 0;        // Ѳ��ģʽ��־ (0=ֹͣѲ�ߣ�1=����Ѳ��)
uint8_t patrolState = 0;        // Ѳ��״̬ (0=�����У�1=Ѳ����)
uint8_t patrolDirection = 0;    // Ѳ�߷��� (0=90��180, 1=180��0, 2=0��180)
float patrolSpeed = 105.0f;       // Ѳ���ٶ� (��/��) Ĭ��45��/�룬2�����90���ƶ�



// �����ȿ��Ʋ���
const float SENSITIVITY_X = 0.3f; // X��������ϵ�� (0.1-1.0)
const float SENSITIVITY_Y = 0.3f; // Y��������ϵ�� (0.1-1.0)

// ��ʼ������
void k230find_Init(void) {
    Serial_Init();
    Servo_Init();
    Servo1_SetAngle(135);
    Servo2_SetAngle(90);
    
    // ��ʼ��Ѳ��״̬
    patrolMode = 1;          // ����������Ѳ��ģʽ
    patrolState = 0;         // �Ȼص�90��λ��
    patrolDirection = 0;     // ��λѲ�߷���
    
    // ����Ŀ��Ƕ�Ϊ90��
    Angle1 = 135.0;
    Angle2 = 87.0;
    TargetAngle1 = 135.0;
    TargetAngle2 = 87.0;
    
    // ��ʼ��Ŀ����״̬
    targetDetected = 0;
    lastTargetUpdateTime = GetSystemTick();
    
    // ��ʼ������״̬����
    moveFlag = 0;
    isBackingUp = 0;
    currentDistance = 0;
    closeStartTime = 0;
    endSignTriggered = 0;
}
uint8_t Serial_GetRxFlag(void) {
    return Serial_RxFlag;  // ֱ�ӷ���ȫ�ֱ�־����
}

// K230Ŀ����º���
void k230find_Update(void) {
		
	uint32_t currentTime = GetSystemTick();
    static uint32_t lastPatrolTime = 0;
    float timeDelta = 0.0f;
	
	// ����ʱ��ֻ����Ҫʱ��
    if (patrolMode || moveFlag) {
        timeDelta = (currentTime - lastPatrolTime) / 1000.0f;
        lastPatrolTime = currentTime;
    }
	
    // 1. ����Ƿ���������
    if (Serial_GetRxFlag() == 1) {			
				
        // �����յ����ֽ�ת��Ϊ����ƫ����
        float deltaX = ((float)Serial_RxPacket[0] - 128.0f) * SENSITIVITY_X;
        float deltaY = ((float)Serial_RxPacket[1] - 128.0f) * SENSITIVITY_Y;
        
        // ����Ŀ��Ƕ�
        TargetAngle1 = Angle1 + deltaX;
        TargetAngle2 = Angle2 + deltaY;
        
	       			
			
        // �Ƕ��޷���0-180�ȣ�
        if (TargetAngle1 < 0.0f) TargetAngle1 = 0.0f;
        if (TargetAngle1 > 270.0f) TargetAngle1 = 270.0f;
        if (TargetAngle2 < 0.0f) TargetAngle2 = 0.0f;
        if (TargetAngle2 > 180.0f) TargetAngle2 = 180.0f;
        
		currentDistance = Serial_RxPacket[2];            
        moveFlag = 1;       // ��ǿ�ʼ�ƶ�
        targetDetected = 1; // ����Ŀ�����־
		// ��⵽Ŀ�꣬�˳�Ѳ��ģʽ
        if (patrolMode) {
            patrolMode = 0;     // �˳�Ѳ��
            patrolState = 0;     // ����Ѳ��״̬
            moveFlag = 1;        // ȷ���ƶ���־������
        }
        
        lastTargetUpdateTime = currentTime;
        Serial_RxFlag = 0;
    }	

        

	// 2. ���Ŀ���Ƿ�ʧ��2�������ݣ�
    if (targetDetected && (currentTime - lastTargetUpdateTime) > TARGET_LOST_TIMEOUT) {
       // ���û���Ŀ��Ƕ�
        TargetAngle1 = 135;
        TargetAngle2 = 87;
        moveFlag = 1;            // �����Ҫ�ƶ�
		
		// ����Ѳ��Ԥ��״̬
        patrolMode = 1;          // ����Ѳ��ģʽ
        patrolState = 0;         // ������Ҫ�ص�90��λ��
        patrolDirection = 0;      // ��λѲ�߷���
        
        targetDetected = 0;      // ����Ŀ�궪ʧ״̬
        
        // ���ý������ʱ��
        closeStartTime = 0;
        endSignTriggered = 0;
    }
						
    // 3. ƽ���ƶ����
    if (moveFlag) {
    const float STEP_ANGLE = 0.5f;  // ��С����ʹ�˶���ƽ��
    
    // �������
    float error1 = fabsf(Angle1 - TargetAngle1);
    float error2 = fabsf(Angle2 - TargetAngle2);
    // X��ƽ���ƶ�
    if (error1 > DEADZONE_ANGLE) {
        if (Angle1 < TargetAngle1) {
            Angle1 += STEP_ANGLE;
            if (Angle1 > TargetAngle1) Angle1 = TargetAngle1;
        } else {
            Angle1 -= STEP_ANGLE;
            if (Angle1 < TargetAngle1) Angle1 = TargetAngle1;
        }
    }
    
    // Y��ƽ���ƶ�
    if (error2 > DEADZONE_ANGLE) {
        if (Angle2 < TargetAngle2) {
            Angle2 += STEP_ANGLE;
            if (Angle2 > TargetAngle2) Angle2 = TargetAngle2;
        } else {
            Angle2 -= STEP_ANGLE;
            if (Angle2 < TargetAngle2) Angle2 = TargetAngle2;
        }
    }
    
    // ����Ƿ񵽴�Ŀ�꣨ʹ��������ֵ��Ϊ�жϱ�׼��
        if (error1 <= DEADZONE_ANGLE && error2 <= DEADZONE_ANGLE) {
            moveFlag = 0;  // �ƶ����
    }
}
    
	// 4. Ѳ��ģʽ����
    /*
    if (patrolMode) {
        // ״̬0���ص�90��λ��
        if (patrolState == 0) {
            // ���û���Ŀ��
            TargetAngle1 = 90;
            TargetAngle2 = 90;
            
            // ���ٻ���
            float returnSpeed = 60.0f; // �����ٶȣ���/�룩
            
            // X�����
            if (fabs(Angle1 - 90) > 0.1f) {
                if (Angle1 < 90) {
                    Angle1 += returnSpeed * timeDelta;
                    if (Angle1 > 90) Angle1 = 90;
                } else {
                    Angle1 -= returnSpeed * timeDelta;
                    if (Angle1 < 90) Angle1 = 90;
                }
            }
            
            // Y�����
            if (fabs(Angle2 - 90) > 0.1f) {
                if (Angle2 < 90) {
                    Angle2 += returnSpeed * timeDelta;
                    if (Angle2 > 90) Angle2 = 90;
                } else {
                    Angle2 -= returnSpeed * timeDelta;
                    if (Angle2 < 90) Angle2 = 90;
                }
            }
            
            // ����Ƿ���ɻ���
            if (fabs(Angle1 - 90) < 0.1f && fabs(Angle2 - 90) < 0.1f) {
                patrolState = 1; // ����Ѳ��״̬
                patrolDirection = 0; // ��90��180��ʼ
            }
        } 
        // ״̬1������Ѳ��ɨ��
        else if (patrolState == 1) {
            // ��ֱ���������90��
            Angle2 = 90;
            
            // ����Ѳ�߷������ˮƽ���
            switch (patrolDirection) {
                case 0: // 90 �� 180
                    Angle1 += patrolSpeed * timeDelta;
                    if (Angle1 >= 180.0f) {
                        Angle1 = 180.0f;
                        patrolDirection = 1; // �л�����180��0
                    }
                    break;
                    
                case 1: // 180 �� 0
                    Angle1 -= patrolSpeed * timeDelta;
                    if (Angle1 <= 0.0f) {
                        Angle1 = 0.0f;
                        patrolDirection = 2; // �л�����0��180
                    }
                    break;
                    
                case 2: // 0 �� 180
                    Angle1 += patrolSpeed * timeDelta;
                    if (Angle1 >= 180.0f) {
                        Angle1 = 180.0f;
                        patrolDirection = 1; // �л���180��0����
                    }
                    break;
            }
            
            // ����ˮƽ�������ֱ����̶�90�ȣ�
            Servo1_SetAngle((uint8_t)Angle1);
            Servo2_SetAngle((uint8_t)Angle2);
            return; // ��������Ķ������
        }
    }	
		*/
		
		//�ص�0�Ȳ���Ѳ��
	  if (patrolMode) {
        // ״̬0���ص�0��λ��
        if (patrolState == 0) {
            // ���û�0Ŀ��
            TargetAngle1 = 0;
            TargetAngle2 = 87;
            
            // ���ٻ���
            float returnSpeed = 60.0f; // �����ٶȣ���/�룩
            
            // X�����
            if (fabs(Angle1 - 0) > 0.1f) {
                if (Angle1 < 0) {
                    Angle1 += returnSpeed * timeDelta;
                    if (Angle1 > 0) Angle1 = 0;
                } else {
                    Angle1 -= returnSpeed * timeDelta;
                    if (Angle1 < 0) Angle1 = 0;
                }
            }
            
            // Y���87
            if (fabs(Angle2 - 87) > 0.1f) {
                if (Angle2 < 87) {
                    Angle2 += returnSpeed * timeDelta;
                    if (Angle2 > 87) Angle2 = 87;
                } else {
                    Angle2 -= returnSpeed * timeDelta;
                    if (Angle2 < 87) Angle2 = 87;
                }
            }
            
            // ����Ƿ���ɻ���
            if (fabs(Angle1 - 0) < 0.1f && fabs(Angle2 - 87) < 0.1f) {
                patrolState = 1; // ����Ѳ��״̬
                patrolDirection = 0; // ��175��0��ʼ
            }
        } 
        // ״̬1������Ѳ��ɨ��
        else if (patrolState == 1) {
            // ��ֱ���������87��
            Angle2 = 87;
            
            // ����Ѳ�߷������ˮƽ���
            switch (patrolDirection) {
                case 0: // 80 �� -50
                    Angle1 -= patrolSpeed * timeDelta;
                    if (Angle1 >= 0.0f) {
                        Angle1 = 0.0f;
                        patrolDirection = 1; // �л�����-50��80
                    }
                    break;
                    
                case 1: // 5 �� 80
                    Angle1 += patrolSpeed * timeDelta;
                    if (Angle1 >= 270.0f) {
                        Angle1 = 270.0f;
                        patrolDirection = 2; // �л�����130��-50
                    }
                    break;
                    
                case 2: // 0 �� 180
                    Angle1 -= patrolSpeed * timeDelta;
                    if (Angle1 <= 0.0f) {
                        Angle1 = 0.0f;
                        patrolDirection = 1; // �л���180��0����
                    }
                    break;

            }

            
            // ����ˮƽ�������ֱ����̶�90�ȣ�
            Servo1_SetAngle((uint8_t)Angle1);
            Servo2_SetAngle((uint8_t)Angle2);
            return; // ��������Ķ������
        }
    }
	// 5. ���¶���Ƕ�
    if (!patrolMode || patrolState == 0) {
        Servo1_SetAngle((uint8_t)Angle1);
        Servo2_SetAngle((uint8_t)Angle2);
    }
	
	
	// 6.��������߼� - ֻ������ʶ��Ŀ��3.5��Ŵ���
      static uint32_t continuousStartTime = 0;  // ������⿪ʼʱ��
      static uint8_t continuousDetected = 0;     // ��������־
    
	// ���״μ�⵽Ŀ��ʱ����ʼ��ʱ
    if (targetDetected && !continuousDetected) {
        continuousStartTime = currentTime;
        continuousDetected = 1;
        }
        
        // ��Ŀ���������ʱ������Ƿ�ﵽ3.5��
    if (targetDetected && continuousDetected) {
       if (!endSignTriggered && (currentTime - continuousStartTime) >= 1000) {
           Endsign_open();         // ���������ź�
           endSignTriggered = 1;
          }
				if((currentTime - continuousStartTime) >= 500)
				{
					GPIO_ResetBits(GPIOB, GPIO_Pin_1);
				}
       }
	
    // ��Ŀ�궪ʧʱ�������������״̬
    if (!targetDetected && continuousDetected) {
        continuousDetected = 0;     // ������������־
        continuousStartTime = 0;    // ���ü�ʱ��
       }
} 

//�����Ƽ������߼�(���������е���)
void START_GO(void) {
 // ִ�й̶���������
 
	Car_GoForward(100);
	Delay_ms(1750); 
    Car_TurnLeft(46);
    Delay_ms(450);
    Car_GoForward(100);
    Delay_ms(1000);
}
// Ŀ����ٵ��̿���1
void TargetTracking_ChassisControl1(void) {
	
	// ֻ�ڼ�⵽Ŀ��ʱִ��
    if (!targetDetected) return;
	
   
    if (targetDetected) {
        
        if ((uint8_t)Angle1 <= 150.0f && (uint8_t)Angle1 >= 30.0f) {
            Car_Stop();
        }
        else if ((uint8_t)Angle1 > 150.0f && (uint8_t)Angle1 <= 180.0f) {
            Car_TurnLeft(20);
        }
        else if ((uint8_t)Angle1 < 30.0f ) {
            Car_TurnRight(20);
        }    
    }     
}

// Ŀ����ٵ��̿���2
void TargetTracking_ChassisControl2(void) {
    if (!targetDetected) return;

    // ����Ŀ����������Ϊ
    if (currentDistance > 40) {
        // ��ӽǶ��˲�����������
        static float filteredAngle = 90.0f;
        filteredAngle = filteredAngle * 0.7f + Angle1 * 0.3f; // ��ͨ�˲�
        
        // ����ת�������-1.0(��ȫ��ת)��+1.0(��ȫ��ת)
        float steerRatio = (filteredAngle - 90.0f) / 90.0f;
        
        // ����������������ǶȽӽ�����ʱ��ת��
        if (fabsf(steerRatio) < (DEAD_ZONE / 90.0f)) {
            steerRatio = 0;
        }
        
        // ���㶯̬�ٶ�
        uint8_t dynamicSpeed = BASE_SPEED;
        if (currentDistance < 80) dynamicSpeed = 28;
        if (currentDistance < 60) dynamicSpeed = 22;
        
        // Ӧ�ò���ת�����
        float speedDiff = fabsf(steerRatio) * dynamicSpeed * DIFF_GAIN;
        speedDiff = (speedDiff < MIN_DIFF) ? MIN_DIFF : speedDiff;
        
        // ������������ǰ������
        GPIO_SetBits(GPIOE, GPIO_Pin_14);
        GPIO_ResetBits(GPIOE, GPIO_Pin_15);
        GPIO_SetBits(GPIOB, GPIO_Pin_11);
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        
        // ����ת�������ò���
        if (steerRatio < -0.001f) { // ��ת
            PWM_SetCompare1(dynamicSpeed - speedDiff); // ��ǰ�ּ���
            PWM_SetCompare2(dynamicSpeed + speedDiff); // ����ּ���
            PWM_SetCompare3(dynamicSpeed - speedDiff); // �Һ��ּ���
            PWM_SetCompare4(dynamicSpeed + speedDiff); // ��ǰ�ּ���
        } 
        else if (steerRatio > 0.001f) { // ��ת
            PWM_SetCompare1(dynamicSpeed + speedDiff); // ��ǰ�ּ���
            PWM_SetCompare2(dynamicSpeed - speedDiff); // ����ּ���
            PWM_SetCompare3(dynamicSpeed + speedDiff); // �Һ��ּ���
            PWM_SetCompare4(dynamicSpeed - speedDiff); // ��ǰ�ּ���
        } 
        else { // ֱ��(��������)
            PWM_SetCompare1(dynamicSpeed);
            PWM_SetCompare2(dynamicSpeed);
            PWM_SetCompare3(dynamicSpeed);
            PWM_SetCompare4(dynamicSpeed);
        }
    } 
    else { // �����40cmʱֹͣ
        Car_Stop();
    }
}
