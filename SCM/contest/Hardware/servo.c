#include "servo.h"


/**
  * @brief  设置舵机脉冲宽度
  * @param  htim: 定时器句柄
  * @param  channel: 定时器通道
  * @param  pulse_us: 脉冲宽度 (微秒)
  */
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    // 计算比较值 (ARR = 定时器周期)
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2; // 假设APB1定时器时钟
    uint32_t prescaler = htim->Instance->PSC;
    uint32_t period = htim->Instance->ARR;
    
    // 计算1μs对应的定时器计数
    uint32_t timer_tick_per_us = (timer_clock / (prescaler + 1)) / 1000000;
    
    // 计算比较值
    uint32_t compare_value = pulse_us * timer_tick_per_us;
    
    // 设置比较值
    switch (channel) {
        case TIM_CHANNEL_1:
            htim->Instance->CCR1 = compare_value;
            break;
        case TIM_CHANNEL_2:
            htim->Instance->CCR2 = compare_value;
            break;
        case TIM_CHANNEL_3:
            htim->Instance->CCR3 = compare_value;
            break;
        case TIM_CHANNEL_4:
            htim->Instance->CCR4 = compare_value;
            break;
        default:
            // 无效通道
            break;
    }

}
