#include "stm32f4xx.h"
#include "Serial.h"
#include "k230.h"
#include "delay.h"
#include <math.h>
#include "motor.h"
#include "system_tick.h"  
#include "endsign.h"
#include "PWM.h"
// 差速转向参数
#define BASE_SPEED 30       // 基础速度值
#define DIFF_GAIN 0.8f      // 降低增益系数(原1.2f)
#define MIN_DIFF 10         // 减小最小速度差(原15)
#define DEAD_ZONE 10.0f      // 死区角度范围(度)
// 舵机角度参数
float Angle1 = 135.0;      // 水平舵机当前角度
float Angle2 = 90.0;      // 垂直舵机当前角度
float TargetAngle1 = 135.0; // 水平舵机目标角度
float TargetAngle2 = 90.0; // 垂直舵机目标角度
uint8_t moveFlag = 0;     // 移动标志（0=停止，1=移动中）
uint8_t isBackingUp = 0;      // 后退状态标志

#define DEADZONE_ANGLE 0.3f // 死区控制阈值（单位：度）
#define TARGET_LOST_TIMEOUT 1000  // 1秒超时(单位ms)
extern volatile uint32_t lastTargetUpdateTime; // 最后收到目标的时间戳

uint8_t currentDistance = 0;  // 存储当前目标距离
uint8_t targetDetected = 0;   // 目标检测标志 (0=无目标，1=有目标)
uint32_t closeStartTime = 0;  // 记录开始小于80cm的时间
uint8_t endSignTriggered = 0; // 标记结束信号是否已触发

// 新增：巡逻状态控制
uint8_t patrolMode = 0;        // 巡逻模式标志 (0=停止巡逻，1=正在巡逻)
uint8_t patrolState = 0;        // 巡逻状态 (0=回中中，1=巡逻中)
uint8_t patrolDirection = 0;    // 巡逻方向 (0=90→180, 1=180→0, 2=0→180)
float patrolSpeed = 105.0f;       // 巡逻速度 (度/秒) 默认45度/秒，2秒完成90度移动



// 灵敏度控制参数
const float SENSITIVITY_X = 0.3f; // X轴灵敏度系数 (0.1-1.0)
const float SENSITIVITY_Y = 0.3f; // Y轴灵敏度系数 (0.1-1.0)

// 初始化函数
void k230find_Init(void) {
    Serial_Init();
    Servo_Init();
    Servo1_SetAngle(135);
    Servo2_SetAngle(90);
    
    // 初始化巡逻状态
    patrolMode = 1;          // 开机即进入巡逻模式
    patrolState = 0;         // 先回到90度位置
    patrolDirection = 0;     // 复位巡逻方向
    
    // 设置目标角度为90度
    Angle1 = 135.0;
    Angle2 = 87.0;
    TargetAngle1 = 135.0;
    TargetAngle2 = 87.0;
    
    // 初始化目标检测状态
    targetDetected = 0;
    lastTargetUpdateTime = GetSystemTick();
    
    // 初始化其他状态变量
    moveFlag = 0;
    isBackingUp = 0;
    currentDistance = 0;
    closeStartTime = 0;
    endSignTriggered = 0;
}
uint8_t Serial_GetRxFlag(void) {
    return Serial_RxFlag;  // 直接返回全局标志变量
}

// K230目标更新函数
void k230find_Update(void) {
		
	uint32_t currentTime = GetSystemTick();
    static uint32_t lastPatrolTime = 0;
    float timeDelta = 0.0f;
	
	// 计算时间差（只在需要时）
    if (patrolMode || moveFlag) {
        timeDelta = (currentTime - lastPatrolTime) / 1000.0f;
        lastPatrolTime = currentTime;
    }
	
    // 1. 检查是否有新数据
    if (Serial_GetRxFlag() == 1) {			
				
        // 将接收到的字节转换为浮点偏移量
        float deltaX = ((float)Serial_RxPacket[0] - 128.0f) * SENSITIVITY_X;
        float deltaY = ((float)Serial_RxPacket[1] - 128.0f) * SENSITIVITY_Y;
        
        // 更新目标角度
        TargetAngle1 = Angle1 + deltaX;
        TargetAngle2 = Angle2 + deltaY;
        
	       			
			
        // 角度限幅（0-180度）
        if (TargetAngle1 < 0.0f) TargetAngle1 = 0.0f;
        if (TargetAngle1 > 270.0f) TargetAngle1 = 270.0f;
        if (TargetAngle2 < 0.0f) TargetAngle2 = 0.0f;
        if (TargetAngle2 > 180.0f) TargetAngle2 = 180.0f;
        
		currentDistance = Serial_RxPacket[2];            
        moveFlag = 1;       // 标记开始移动
        targetDetected = 1; // 设置目标检测标志
		// 检测到目标，退出巡逻模式
        if (patrolMode) {
            patrolMode = 0;     // 退出巡逻
            patrolState = 0;     // 重置巡逻状态
            moveFlag = 1;        // 确保移动标志被设置
        }
        
        lastTargetUpdateTime = currentTime;
        Serial_RxFlag = 0;
    }	

        

	// 2. 检测目标是否丢失（2秒无数据）
    if (targetDetected && (currentTime - lastTargetUpdateTime) > TARGET_LOST_TIMEOUT) {
       // 设置回正目标角度
        TargetAngle1 = 135;
        TargetAngle2 = 87;
        moveFlag = 1;            // 标记需要移动
		
		// 进入巡逻预备状态
        patrolMode = 1;          // 进入巡逻模式
        patrolState = 0;         // 首先需要回到90度位置
        patrolDirection = 0;      // 复位巡逻方向
        
        targetDetected = 0;      // 进入目标丢失状态
        
        // 重置近距离计时器
        closeStartTime = 0;
        endSignTriggered = 0;
    }
						
    // 3. 平滑移动舵机
    if (moveFlag) {
    const float STEP_ANGLE = 0.5f;  // 减小步长使运动更平滑
    
    // 计算误差
    float error1 = fabsf(Angle1 - TargetAngle1);
    float error2 = fabsf(Angle2 - TargetAngle2);
    // X轴平滑移动
    if (error1 > DEADZONE_ANGLE) {
        if (Angle1 < TargetAngle1) {
            Angle1 += STEP_ANGLE;
            if (Angle1 > TargetAngle1) Angle1 = TargetAngle1;
        } else {
            Angle1 -= STEP_ANGLE;
            if (Angle1 < TargetAngle1) Angle1 = TargetAngle1;
        }
    }
    
    // Y轴平滑移动
    if (error2 > DEADZONE_ANGLE) {
        if (Angle2 < TargetAngle2) {
            Angle2 += STEP_ANGLE;
            if (Angle2 > TargetAngle2) Angle2 = TargetAngle2;
        } else {
            Angle2 -= STEP_ANGLE;
            if (Angle2 < TargetAngle2) Angle2 = TargetAngle2;
        }
    }
    
    // 检查是否到达目标（使用死区阈值作为判断标准）
        if (error1 <= DEADZONE_ANGLE && error2 <= DEADZONE_ANGLE) {
            moveFlag = 0;  // 移动完成
    }
}
    
	// 4. 巡逻模式处理
    /*
    if (patrolMode) {
        // 状态0：回到90度位置
        if (patrolState == 0) {
            // 设置回中目标
            TargetAngle1 = 90;
            TargetAngle2 = 90;
            
            // 快速回中
            float returnSpeed = 60.0f; // 回中速度（度/秒）
            
            // X轴回中
            if (fabs(Angle1 - 90) > 0.1f) {
                if (Angle1 < 90) {
                    Angle1 += returnSpeed * timeDelta;
                    if (Angle1 > 90) Angle1 = 90;
                } else {
                    Angle1 -= returnSpeed * timeDelta;
                    if (Angle1 < 90) Angle1 = 90;
                }
            }
            
            // Y轴回中
            if (fabs(Angle2 - 90) > 0.1f) {
                if (Angle2 < 90) {
                    Angle2 += returnSpeed * timeDelta;
                    if (Angle2 > 90) Angle2 = 90;
                } else {
                    Angle2 -= returnSpeed * timeDelta;
                    if (Angle2 < 90) Angle2 = 90;
                }
            }
            
            // 检查是否完成回中
            if (fabs(Angle1 - 90) < 0.1f && fabs(Angle2 - 90) < 0.1f) {
                patrolState = 1; // 进入巡逻状态
                patrolDirection = 0; // 从90→180开始
            }
        } 
        // 状态1：进行巡逻扫描
        else if (patrolState == 1) {
            // 垂直舵机保持在90度
            Angle2 = 90;
            
            // 根据巡逻方向更新水平舵机
            switch (patrolDirection) {
                case 0: // 90 → 180
                    Angle1 += patrolSpeed * timeDelta;
                    if (Angle1 >= 180.0f) {
                        Angle1 = 180.0f;
                        patrolDirection = 1; // 切换方向：180→0
                    }
                    break;
                    
                case 1: // 180 → 0
                    Angle1 -= patrolSpeed * timeDelta;
                    if (Angle1 <= 0.0f) {
                        Angle1 = 0.0f;
                        patrolDirection = 2; // 切换方向：0→180
                    }
                    break;
                    
                case 2: // 0 → 180
                    Angle1 += patrolSpeed * timeDelta;
                    if (Angle1 >= 180.0f) {
                        Angle1 = 180.0f;
                        patrolDirection = 1; // 切换回180→0方向
                    }
                    break;
            }
            
            // 更新水平舵机（垂直舵机固定90度）
            Servo1_SetAngle((uint8_t)Angle1);
            Servo2_SetAngle((uint8_t)Angle2);
            return; // 跳过后面的舵机更新
        }
    }	
		*/
		
		//回到0度并且巡航
	  if (patrolMode) {
        // 状态0：回到0度位置
        if (patrolState == 0) {
            // 设置回0目标
            TargetAngle1 = 0;
            TargetAngle2 = 87;
            
            // 快速回中
            float returnSpeed = 60.0f; // 回中速度（度/秒）
            
            // X轴回零
            if (fabs(Angle1 - 0) > 0.1f) {
                if (Angle1 < 0) {
                    Angle1 += returnSpeed * timeDelta;
                    if (Angle1 > 0) Angle1 = 0;
                } else {
                    Angle1 -= returnSpeed * timeDelta;
                    if (Angle1 < 0) Angle1 = 0;
                }
            }
            
            // Y轴回87
            if (fabs(Angle2 - 87) > 0.1f) {
                if (Angle2 < 87) {
                    Angle2 += returnSpeed * timeDelta;
                    if (Angle2 > 87) Angle2 = 87;
                } else {
                    Angle2 -= returnSpeed * timeDelta;
                    if (Angle2 < 87) Angle2 = 87;
                }
            }
            
            // 检查是否完成回中
            if (fabs(Angle1 - 0) < 0.1f && fabs(Angle2 - 87) < 0.1f) {
                patrolState = 1; // 进入巡逻状态
                patrolDirection = 0; // 从175→0开始
            }
        } 
        // 状态1：进行巡逻扫描
        else if (patrolState == 1) {
            // 垂直舵机保持在87度
            Angle2 = 87;
            
            // 根据巡逻方向更新水平舵机
            switch (patrolDirection) {
                case 0: // 80 → -50
                    Angle1 -= patrolSpeed * timeDelta;
                    if (Angle1 >= 0.0f) {
                        Angle1 = 0.0f;
                        patrolDirection = 1; // 切换方向：-50→80
                    }
                    break;
                    
                case 1: // 5 → 80
                    Angle1 += patrolSpeed * timeDelta;
                    if (Angle1 >= 270.0f) {
                        Angle1 = 270.0f;
                        patrolDirection = 2; // 切换方向：130→-50
                    }
                    break;
                    
                case 2: // 0 → 180
                    Angle1 -= patrolSpeed * timeDelta;
                    if (Angle1 <= 0.0f) {
                        Angle1 = 0.0f;
                        patrolDirection = 1; // 切换回180→0方向
                    }
                    break;

            }

            
            // 更新水平舵机（垂直舵机固定90度）
            Servo1_SetAngle((uint8_t)Angle1);
            Servo2_SetAngle((uint8_t)Angle2);
            return; // 跳过后面的舵机更新
        }
    }
	// 5. 更新舵机角度
    if (!patrolMode || patrolState == 0) {
        Servo1_SetAngle((uint8_t)Angle1);
        Servo2_SetAngle((uint8_t)Angle2);
    }
	
	
	// 6.连续检测逻辑 - 只有连续识别到目标3.5秒才触发
      static uint32_t continuousStartTime = 0;  // 连续检测开始时间
      static uint8_t continuousDetected = 0;     // 连续检测标志
    
	// 当首次检测到目标时，开始计时
    if (targetDetected && !continuousDetected) {
        continuousStartTime = currentTime;
        continuousDetected = 1;
        }
        
        // 当目标持续存在时，检查是否达到3.5秒
    if (targetDetected && continuousDetected) {
       if (!endSignTriggered && (currentTime - continuousStartTime) >= 1000) {
           Endsign_open();         // 触发结束信号
           endSignTriggered = 1;
          }
				if((currentTime - continuousStartTime) >= 500)
				{
					GPIO_ResetBits(GPIOB, GPIO_Pin_1);
				}
       }
	
    // 当目标丢失时，重置连续检测状态
    if (!targetDetected && continuousDetected) {
        continuousDetected = 0;     // 重置连续检测标志
        continuousStartTime = 0;    // 重置计时器
       }
} 

//比赛推荐控制逻辑(参数可自行调节)
void START_GO(void) {
 // 执行固定动作序列
 
	Car_GoForward(100);
	Delay_ms(1750); 
    Car_TurnLeft(46);
    Delay_ms(450);
    Car_GoForward(100);
    Delay_ms(1000);
}
// 目标跟踪底盘控制1
void TargetTracking_ChassisControl1(void) {
	
	// 只在检测到目标时执行
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

// 目标跟踪底盘控制2
void TargetTracking_ChassisControl2(void) {
    if (!targetDetected) return;

    // 根据目标距离调整行为
    if (currentDistance > 40) {
        // 添加角度滤波和死区处理
        static float filteredAngle = 90.0f;
        filteredAngle = filteredAngle * 0.7f + Angle1 * 0.3f; // 低通滤波
        
        // 计算转向比例：-1.0(完全左转)到+1.0(完全右转)
        float steerRatio = (filteredAngle - 90.0f) / 90.0f;
        
        // 添加中央死区：当角度接近中心时不转向
        if (fabsf(steerRatio) < (DEAD_ZONE / 90.0f)) {
            steerRatio = 0;
        }
        
        // 计算动态速度
        uint8_t dynamicSpeed = BASE_SPEED;
        if (currentDistance < 80) dynamicSpeed = 28;
        if (currentDistance < 60) dynamicSpeed = 22;
        
        // 应用差速转向控制
        float speedDiff = fabsf(steerRatio) * dynamicSpeed * DIFF_GAIN;
        speedDiff = (speedDiff < MIN_DIFF) ? MIN_DIFF : speedDiff;
        
        // 设置所有轮子前进方向
        GPIO_SetBits(GPIOE, GPIO_Pin_14);
        GPIO_ResetBits(GPIOE, GPIO_Pin_15);
        GPIO_SetBits(GPIOB, GPIO_Pin_11);
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        
        // 根据转向方向设置差速
        if (steerRatio < -0.001f) { // 左转
            PWM_SetCompare1(dynamicSpeed - speedDiff); // 右前轮加速
            PWM_SetCompare2(dynamicSpeed + speedDiff); // 左后轮减速
            PWM_SetCompare3(dynamicSpeed - speedDiff); // 右后轮加速
            PWM_SetCompare4(dynamicSpeed + speedDiff); // 左前轮减速
        } 
        else if (steerRatio > 0.001f) { // 右转
            PWM_SetCompare1(dynamicSpeed + speedDiff); // 右前轮减速
            PWM_SetCompare2(dynamicSpeed - speedDiff); // 左后轮加速
            PWM_SetCompare3(dynamicSpeed + speedDiff); // 右后轮减速
            PWM_SetCompare4(dynamicSpeed - speedDiff); // 左前轮加速
        } 
        else { // 直行(中央死区)
            PWM_SetCompare1(dynamicSpeed);
            PWM_SetCompare2(dynamicSpeed);
            PWM_SetCompare3(dynamicSpeed);
            PWM_SetCompare4(dynamicSpeed);
        }
    } 
    else { // 距离≤40cm时停止
        Car_Stop();
    }
}
