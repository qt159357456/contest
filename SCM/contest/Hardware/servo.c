#include "servo.h"


/**
  * @brief  ���ö��������
  * @param  htim: ��ʱ�����
  * @param  channel: ��ʱ��ͨ��
  * @param  pulse_us: ������ (΢��)
  */
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    // ����Ƚ�ֵ (ARR = ��ʱ������)
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2; // ����APB1��ʱ��ʱ��
    uint32_t prescaler = htim->Instance->PSC;
    uint32_t period = htim->Instance->ARR;
    
    // ����1��s��Ӧ�Ķ�ʱ������
    uint32_t timer_tick_per_us = (timer_clock / (prescaler + 1)) / 1000000;
    
    // ����Ƚ�ֵ
    uint32_t compare_value = pulse_us * timer_tick_per_us;
    
    // ���ñȽ�ֵ
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
            // ��Чͨ��
            break;
    }

}
