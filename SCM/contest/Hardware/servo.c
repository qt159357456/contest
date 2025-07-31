#include "servo.h"


/**
  * @brief  设置舵机脉冲宽度
  * @param  htim: 定时器句柄
  * @param  channel: 定时器通道
  * @param  pulse_us: 脉冲宽度 (微秒)
  */
	
int test;
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    // 安全校验：确保脉冲在有效范围内
//    if(pulse_us < SERVO_MIN_PULSE) pulse_us = SERVO_MIN_PULSE;
//    if(pulse_us > SERVO_MAX_PULSE) pulse_us = SERVO_MAX_PULSE;
    
    // 修正时钟计算逻辑
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq()*2;
		test = pulse_us * (timer_clock / (htim->Instance->PSC + 1)) / 1000000;
		
    
    // 使用安全宏设置比较值
    switch (channel) {
        case TIM_CHANNEL_1:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 
                pulse_us);
            break;
        case TIM_CHANNEL_2:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 
                pulse_us);
            break;
        case TIM_CHANNEL_3:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 
                pulse_us);
            break;
        case TIM_CHANNEL_4:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 
                pulse_us);
            break;
        default:
            break;
    }
}

void Servo_SetPWM(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse)
{    
    // 使用安全宏设置比较值
    switch (channel) {
        case TIM_CHANNEL_1:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,pulse);
            break;
        case TIM_CHANNEL_2:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2,pulse);
            break;
        case TIM_CHANNEL_3:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3,pulse);
            break;
        case TIM_CHANNEL_4:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4,pulse);
            break;
        default:
            break;
    }
}


