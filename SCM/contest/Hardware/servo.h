#ifndef __SERVO_H
#define __SERVO_H


#include "stm32f1xx_hal.h"

// Ĭ�϶������
#define SERVO_MIN_PULSE      500     // 0.5ms
#define SERVO_MAX_PULSE      2500    // 2.5ms

/* ������ýṹ�� */
typedef struct {
    TIM_HandleTypeDef *htim;     // ��ʱ�����
    uint32_t channel;            // PWMͨ��
    uint16_t min_pulse;          // ��С������ (��s)
    uint16_t max_pulse;          // ��������� (��s)
    float min_angle;             // ��С�Ƕ� (��)
    float max_angle;             // ���Ƕ� (��)
} Servo_Config;

/* �������� */
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us);
void Servo_SetPWM(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse);
#endif /* __SERVO_H */
