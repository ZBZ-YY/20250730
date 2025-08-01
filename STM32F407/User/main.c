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
extern uint8_t targetDetected; // 声明目标检测标志
volatile uint32_t lastTargetUpdateTime = 0;

// 新增全局变量
uint32_t avoidObstacleStartTime = 0; // 避障开始时间
uint8_t isAvoidingObstacle = 0;      // 是否正在避障

// 按键初始化函数 - 针对三引脚按键
void Key_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. 使能GPIOD时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    // 2. 配置PD12为输入模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        // 输入模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出（虽然用于输入）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 高速
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // 无上拉下拉（按键模块自带）
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

// 按键检测函数 - 针对三引脚按键
uint8_t Key_GetState(void) {
    // 直接读取OUT引脚状态
    return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12);
}

int main(void)
{
	
    // 初始化系统
    Endsign_Init();
    SystemTick_Init(); // 添加系统滴答定时器初始化
    lastTargetUpdateTime = GetSystemTick(); // 添加这行初始化
    Motor_Init(); // 电机初始化
    EXTI_INFRAREDAVOID1_Config(); // 红外1中断
    EXTI_INFRAREDAVOID2_Config(); // 红外2中断
    EXTI_INFRAREDAVOID3_Config(); // 红外3中断
    EXTI_INFRAREDAVOID4_Config(); // 红外4中断 
    
    // 初始化按键
    Key_Init();
    
    // 初始化避障状态变量
    avoidObstacleStartTime = 0;
    isAvoidingObstacle = 0;
    
    // 等待按键按下 - 确保系统暂停在这里
    while (1) {
        // 按键按下时OUT引脚为高电平
        if (Key_GetState() == 0) {
            // 添加延时确认按键
            Delay_ms(50);
            if (Key_GetState() == 0) {
                break; // 确认按键按下
            }
        }
        // 添加短暂延时减少CPU占用
        Delay_ms(10);
    }
    
	  //比赛推荐运动控制逻辑
    START_GO();
    
    // 初始化K230跟踪系统
    k230find_Init();
    

  // 主循环
while(1)
{

    k230find_Update();
    
    uint32_t currentTime = GetSystemTick();
    
    // 1. 检查避障状态是否需要重置
    if (isAvoidingObstacle && (currentTime - avoidObstacleStartTime) >= 500) {
        isAvoidingObstacle = 0; // 0.5秒避障结束
    }
    
    // 2. 一次性读取所有传感器状态（仅使用四个传感器）
    uint32_t sensorData = GPIO_ReadInputData(GPIOD);
    uint8_t right_clear = (sensorData & GPIO_Pin_8) != 0;  // 右
    uint8_t forwardright_clear = (sensorData & GPIO_Pin_7) != 0; // 右前
    uint8_t left_clear = (sensorData & GPIO_Pin_9) != 0;   // 左
    uint8_t forwardleft_clear = (sensorData & GPIO_Pin_6) != 0;  // 左前
    
    // 3. 障碍检测
    uint8_t obstacleDetected = 0;
    uint8_t turnRightFlag = 0;  // 右转标志
    uint8_t turnLeftFlag = 0;   // 左转标志
    uint8_t stopFlag = 0;       // 停止标志
    
    // 计算障碍物数量（仅四个传感器）
    int obstacleCount = !right_clear + !forwardright_clear + 
                        !left_clear + !forwardleft_clear;
    
    if (obstacleCount > 0) {
        obstacleDetected = 1;
        
        // 记录避障开始时间
        if (!isAvoidingObstacle) {
            isAvoidingObstacle = 1;
            avoidObstacleStartTime = currentTime;
        }
        
        // 规则1: 所有四个传感器都检测到障碍 - 停止
        if (obstacleCount == 4) {
            stopFlag = 1;
        }
        // 规则2: 三个传感器检测到障碍
        else if (obstacleCount == 3) {
            // 计算左右两侧障碍数量
            int leftObstacleCount = !left_clear + !forwardleft_clear;
            int rightObstacleCount = !right_clear + !forwardright_clear;
            
            if (leftObstacleCount > rightObstacleCount) {
                // 左侧障碍多 -> 向右转
                turnRightFlag = 1;
            } else {
                // 右侧障碍多 -> 向左转
                turnLeftFlag = 1;
            }
        }
        // 规则3: 两个传感器检测到障碍
        else if (obstacleCount == 2) {
            // 1. 左前和左都有障碍 -> 右转
            if (!forwardleft_clear && !left_clear) {
                turnRightFlag = 1;
            }
            // 2. 右前和右都有障碍 -> 左转
            else if (!forwardright_clear && !right_clear) {
                turnLeftFlag = 1;
            }
            // 3. 左前和右都有障碍 -> 左转
            else if (!forwardleft_clear && !right_clear) {
                turnLeftFlag = 1;
            }
            // 4. 右前和左都有障碍 -> 右转
            else if (!forwardright_clear && !left_clear) {
                turnRightFlag = 1;
            }
            // 5. 左前和右前都有障碍 -> 根据方向优先级处理
            else if (!forwardleft_clear && !forwardright_clear) {
                // 前方有障碍，根据左右传感器状态决定
                if (!left_clear) {
                    turnRightFlag = 1;  // 左边有障碍，右转
                } else if (!right_clear) {
                    turnLeftFlag = 1;   // 右边有障碍，左转
                } else {
                    // 左右都没有障碍，随机选一个方向（这里选择右转）
                    turnRightFlag = 1;
                }
            }
            // 6. 左侧和右侧都有障碍 -> 左转
            else if (!left_clear && !right_clear) {
                turnLeftFlag = 1;
            }
            // 7. 其他未明确组合 - 安全停止
            else {
                stopFlag = 1;
            }
        }
        // 规则4: 一个传感器检测到障碍
        else if (obstacleCount == 1) {
            // 左侧或左前有障碍 -> 右转
            if (!left_clear || !forwardleft_clear) {
                turnRightFlag = 1;
            }
            // 右侧或右前有障碍 -> 左转
            else if (!right_clear || !forwardright_clear) {
                turnLeftFlag = 1;
            }
        }
    }
    
    // 4. 执行避障动作
    if (obstacleDetected) {
        if (stopFlag) {
            Car_Stop();
        }
        else if (turnRightFlag) {

            Car_TurnRight(80); // 1 = 右转
        }
        else if (turnLeftFlag) {
            Car_TurnLeft(80); // 0 = 左转
        }
        else {
            Car_Stop();
        }
    }
    else {
        // 无障碍 - 重置避障标志
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
