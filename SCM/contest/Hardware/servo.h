#ifndef __SERVO_H
#define __SERVO_H


#include "stm32f1xx_hal.h"

// 默认舵机参数
#define SERVO_MIN_PULSE      50     // 0.5ms
#define SERVO_MAX_PULSE      250    // 2.5ms

/* 舵机配置结构体 */
typedef struct {
    TIM_HandleTypeDef *htim;     // 定时器句柄
    uint32_t channel;            // PWM通道
    uint16_t min_pulse;          // 最小脉冲宽度 (μs)
    uint16_t max_pulse;          // 最大脉冲宽度 (μs)
    float min_angle;             // 最小角度 (°)
    float max_angle;             // 最大角度 (°)
} Servo_Config;

/* 函数声明 */
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us);
void Servo_Calibrate(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t min_pulse, uint16_t max_pulse);

#endif /* __SERVO_H */
