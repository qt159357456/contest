#include "servo.h"


/**
  * @brief  ���ö��������
  * @param  htim: ��ʱ�����
  * @param  channel: ��ʱ��ͨ��
  * @param  pulse_us: ������ (΢��)
  */
	
int test;
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    // ��ȫУ�飺ȷ����������Ч��Χ��
//    if(pulse_us < SERVO_MIN_PULSE) pulse_us = SERVO_MIN_PULSE;
//    if(pulse_us > SERVO_MAX_PULSE) pulse_us = SERVO_MAX_PULSE;
    
    // ����ʱ�Ӽ����߼�
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq()*2;
		test = pulse_us * (timer_clock / (htim->Instance->PSC + 1)) / 1000000;
		
    
    // ʹ�ð�ȫ�����ñȽ�ֵ
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
    // ʹ�ð�ȫ�����ñȽ�ֵ
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


